"""Competition task schema, question generation, and strict parsers."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import random
import re
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple


SCHEMA_VERSION = "1.0"
VALID_COLORS = ("red", "blue")
VALID_ZONES = ("A", "B", "C")
VALID_SOURCES = ("llm", "rules")
MAXIMUM_TOTAL_CUBES = 5

_COLOR_NAMES = {
    "红": "red",
    "红色": "red",
    "蓝": "blue",
    "蓝色": "blue",
    "red": "red",
    "blue": "blue",
}
_COLOR_LABELS = {"red": "红色", "blue": "蓝色"}
_CHINESE_NUMBERS = {
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
    "十": 10,
}
_COUNT_TOKEN = r"\d+|一|二|两|三|四|五|六|七|八|九|十"
_DESTINATION = r"送到|搬运到|放到|投放到|运到|放入|到"
_CHINESE_COUNT_FIRST = re.compile(
    rf"(?P<count>{_COUNT_TOKEN})\s*个?\s*"
    rf"(?P<color>红色|蓝色|红|蓝)\s*方块.*?"
    rf"(?:{_DESTINATION})\s*(?P<zone>[ABCabc])\s*区"
)
_CHINESE_COLOR_FIRST = re.compile(
    rf"(?P<color>红色|蓝色|红|蓝)\s*方块\s*"
    rf"(?P<count>{_COUNT_TOKEN})\s*个?.*?"
    rf"(?:{_DESTINATION})\s*(?P<zone>[ABCabc])\s*区"
)
_ENGLISH_TASK = re.compile(
    r"(?P<count>\d+)\s*(?P<color>red|blue)\s*"
    r"(?:cubes?|blocks?).*?(?:to|into)\s*"
    r"(?:zone\s*)?(?P<zone>[ABCabc])\b",
    re.IGNORECASE,
)


class TaskProtocolError(ValueError):
    """Raised when a task cannot satisfy the competition schema."""


@dataclass(frozen=True)
class TaskItem:
    """One ordered color, quantity, and destination instruction."""

    color: str
    count: int
    target_zone: str

    def __post_init__(self) -> None:
        color = str(self.color).lower()
        zone = str(self.target_zone).upper()
        if color not in VALID_COLORS:
            raise TaskProtocolError(f"Unsupported color: {self.color}")
        if isinstance(self.count, bool) or not isinstance(self.count, int):
            raise TaskProtocolError("Task count must be an integer")
        if not 1 <= self.count <= 10:
            raise TaskProtocolError("Task count must be between 1 and 10")
        if zone not in VALID_ZONES:
            raise TaskProtocolError(
                f"Unsupported target zone: {self.target_zone}"
            )
        object.__setattr__(self, "color", color)
        object.__setattr__(self, "target_zone", zone)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-compatible instruction."""
        return {
            "color": self.color,
            "count": self.count,
            "target_zone": self.target_zone,
        }


@dataclass(frozen=True)
class TaskPlan:
    """Validated, ordered task plan shared by all competition nodes."""

    task_id: str
    source: str
    items: Tuple[TaskItem, ...]
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise TaskProtocolError(
                f"Unsupported schema version: {self.schema_version}"
            )
        if not self.task_id or not isinstance(self.task_id, str):
            raise TaskProtocolError("task_id must be a non-empty string")
        if self.source not in VALID_SOURCES:
            raise TaskProtocolError(f"Unsupported source: {self.source}")
        if not self.items:
            raise TaskProtocolError("Task plan must contain at least one item")
        if len(self.items) > 6:
            raise TaskProtocolError("Task plan contains too many items")
        if not all(isinstance(item, TaskItem) for item in self.items):
            raise TaskProtocolError("Task items must be TaskItem instances")
        total = sum(item.count for item in self.items)
        if total > MAXIMUM_TOTAL_CUBES:
            raise TaskProtocolError(
                "Task requests more than "
                f"{MAXIMUM_TOTAL_CUBES} cubes in one round"
            )

    @property
    def counts(self) -> Dict[str, int]:
        """Return total requested quantities in stable red-blue order."""
        totals = {color: 0 for color in VALID_COLORS}
        for item in self.items:
            totals[item.color] += item.count
        return totals

    @property
    def execution_order(self) -> Tuple[str, ...]:
        """Expand ordered task items into one color entry per cube."""
        return tuple(
            item.color
            for item in self.items
            for _ in range(item.count)
        )

    def to_dict(self) -> Dict[str, Any]:
        """Return the complete public task message."""
        return {
            "schema_version": self.schema_version,
            "task_id": self.task_id,
            "source": self.source,
            "items": [item.to_dict() for item in self.items],
            "counts": self.counts,
            "execution_order": list(self.execution_order),
        }

    def to_json(self) -> str:
        """Serialize the plan deterministically for a ROS String topic."""
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )


def make_task_id(question: str) -> str:
    """Create a stable short identifier from an exact task question."""
    normalized = " ".join(question.split())
    if not normalized:
        raise TaskProtocolError("Task question is empty")
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:16]


def _parse_count(token: str) -> int:
    if token.isdigit():
        return int(token)
    try:
        return _CHINESE_NUMBERS[token]
    except KeyError as exception:
        raise TaskProtocolError(
            f"Unsupported quantity token: {token}"
        ) from exception


def _items_from_matches(
    matches: Iterable[re.Match],
) -> Tuple[TaskItem, ...]:
    ordered = sorted(matches, key=lambda match: match.start())
    items = []
    occupied_ranges = []
    for match in ordered:
        span = match.span()
        if any(
            span[0] < existing[1] and existing[0] < span[1]
            for existing in occupied_ranges
        ):
            continue
        color_token = match.group("color").lower()
        items.append(
            TaskItem(
                color=_COLOR_NAMES[color_token],
                count=_parse_count(match.group("count")),
                target_zone=match.group("zone"),
            )
        )
        occupied_ranges.append(span)
    return tuple(items)


def parse_task_question(
    question: str,
    source: str = "rules",
) -> TaskPlan:
    """Parse supported Chinese or English competition wording."""
    if not isinstance(question, str) or not question.strip():
        raise TaskProtocolError("Task question is empty")
    matches = []
    for pattern in (
        _CHINESE_COUNT_FIRST,
        _CHINESE_COLOR_FIRST,
        _ENGLISH_TASK,
    ):
        matches.extend(pattern.finditer(question))
    items = _items_from_matches(matches)
    if not items:
        raise TaskProtocolError(
            "Question does not contain a complete color/count/zone clause"
        )
    return TaskPlan(
        task_id=make_task_id(question),
        source=source,
        items=items,
    )


def _extract_json_object(content: str) -> Mapping[str, Any]:
    if not isinstance(content, str) or not content.strip():
        raise TaskProtocolError("LLM response is empty")
    stripped = content.strip()
    if stripped.startswith("```"):
        stripped = re.sub(r"^```(?:json)?\s*", "", stripped)
        stripped = re.sub(r"\s*```$", "", stripped)
    start = stripped.find("{")
    end = stripped.rfind("}")
    if start < 0 or end <= start:
        raise TaskProtocolError("LLM response does not contain a JSON object")
    try:
        value = json.loads(stripped[start:end + 1])
    except json.JSONDecodeError as exception:
        raise TaskProtocolError(
            f"LLM response contains invalid JSON: {exception.msg}"
        ) from exception
    if not isinstance(value, Mapping):
        raise TaskProtocolError("LLM response root must be a JSON object")
    return value


def _item_from_mapping(value: Any) -> TaskItem:
    if not isinstance(value, Mapping):
        raise TaskProtocolError("Each LLM task item must be an object")
    allowed = {"color", "count", "target_zone"}
    if set(value) != allowed:
        raise TaskProtocolError(
            "Each LLM item must contain only color, count, target_zone"
        )
    return TaskItem(
        color=value["color"],
        count=value["count"],
        target_zone=value["target_zone"],
    )


def parse_llm_response(content: str, question: str) -> TaskPlan:
    """Validate LLM JSON and cross-check it against parseable wording."""
    value = _extract_json_object(content)
    allowed = {"schema_version", "items"}
    if not set(value).issubset(allowed) or "items" not in value:
        raise TaskProtocolError(
            "LLM root must contain only schema_version and items"
        )
    version = value.get("schema_version", SCHEMA_VERSION)
    raw_items = value["items"]
    if not isinstance(raw_items, Sequence) or isinstance(raw_items, str):
        raise TaskProtocolError("LLM items must be a JSON array")
    items = tuple(_item_from_mapping(item) for item in raw_items)
    plan = TaskPlan(
        task_id=make_task_id(question),
        source="llm",
        items=items,
        schema_version=version,
    )

    try:
        rules_plan = parse_task_question(question)
    except TaskProtocolError:
        return plan
    if plan.items != rules_plan.items:
        raise TaskProtocolError(
            "LLM plan disagrees with color/count/zone values in question"
        )
    return plan


def parse_with_fallback(
    question: str,
    llm_content: Optional[str],
) -> TaskPlan:
    """Prefer valid LLM output and otherwise use deterministic rules."""
    if llm_content is not None:
        try:
            return parse_llm_response(llm_content, question)
        except TaskProtocolError:
            pass
    return parse_task_question(question, source="rules")


def generate_task_question(randomizer: random.Random) -> str:
    """Generate one five-cube, rules-parseable PDF-style question."""
    colors = list(VALID_COLORS)
    randomizer.shuffle(colors)
    zones = list(VALID_ZONES)
    randomizer.shuffle(zones)
    first_count = randomizer.randint(1, MAXIMUM_TOTAL_CUBES - 1)
    second_count = MAXIMUM_TOTAL_CUBES - first_count
    first_color, second_color = colors
    first_zone, second_zone = zones[:2]
    templates = (
        (
            "请先将{first_count}个{first_color}方块送到{first_zone}区，"
            "再将{second_count}个{second_color}方块送到{second_zone}区。"
        ),
        (
            "任务：先搬运{first_count}个{first_color}方块到{first_zone}区，"
            "然后搬运{second_count}个{second_color}方块到"
            "{second_zone}区。"
        ),
    )
    template = randomizer.choice(templates)
    return template.format(
        first_count=first_count,
        first_color=_COLOR_LABELS[first_color],
        first_zone=first_zone,
        second_count=second_count,
        second_color=_COLOR_LABELS[second_color],
        second_zone=second_zone,
    )
