"""
Deterministic solver for competition problem-generator word problems.

The official generator (TMSCQtest) emits arithmetic word problems such as the
wardrobe example from the 2026 rules PDF:

    "每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;
     小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。
     设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"

The solver computes the two variables, then applies the pre-match mapping
rules (e.g. "x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区") to
build the exact TaskPlan the mission coordinator already consumes.
"""

from __future__ import annotations

import dataclasses
import json
import math
import re
from typing import Dict, Mapping, Optional, Sequence, Tuple

from tools_demo.task_protocol import make_task_id
from tools_demo.task_protocol import TaskItem
from tools_demo.task_protocol import TaskPlan
from tools_demo.task_protocol import TaskProtocolError


_CAPACITY_RE = re.compile(
    r"每个(?:[^;；。]+?)\s*有\s*(\d+)\s*(?:件|只|个|台|辆|箱|盒|袋|瓶|包|罐|份|组|本)"
)
_ITEM_TYPES_RE = re.compile(
    r"有\s*([^,，;；。、]{1,8})、([^,，;；。、]{1,8})\s*两种"
)
_VAR_DECL_RE = re.compile(
    r"设\s*([^,，;；。]+?)\s*(?:数量|数)为\s*x\s*[,，]?\s*"
    r"([^,，;；。]+?)\s*(?:数量|数)为\s*y"
)
_QUANTITY_UNIT = r"(?:件|条|只|个|台|辆|箱|张|双|盒|袋|瓶|包|罐|份|组|本)"
_DEMAND_RE = re.compile(
    r"(\d+)\s*" + _QUANTITY_UNIT + r"\s*([^,，、;；。0-9]+)"
)
# Known verbs are retained only to support non-generator names such as
# ``甲需要...``. Official generator clauses are recognized structurally from
# their stable ``小X[、小Y都] + action + quantity + item`` shape, so a new
# action word cannot block a valid round before the model is even called.
_DEMAND_VERB = r"(?:需要|领养|想要|想玩|借阅|领取|要)"
_KNOWN_SUBJECT_PREFIX_RE = re.compile(r"(.+?)" + _DEMAND_VERB + r"\s*")
_GENERATOR_SUBJECT_PREFIX_RE = re.compile(
    r"(?P<subjects>小[\u4e00-\u9fff]"
    r"(?:\s*(?:[、，,]|和|与)\s*小[\u4e00-\u9fff])*)"
    r"(?:\s*都)?(?P<action>.+)"
)

# --- mapping rules (free text, Chinese/English punctuation tolerant) --------

_COLOR_WORD = r"红色|蓝色|红|蓝"
_COLOR_NAMES = {
    "红": "red",
    "红色": "red",
    "蓝": "blue",
    "蓝色": "blue",
}
_VAR_COLOR_RE = re.compile(
    r"([xy])\s*(?:代表|对应|为|是|: |：|->|→|=)\s*(" + _COLOR_WORD + r")"
)
_ZONE_GE_RE = re.compile(
    r"(?:数量|数值)\s*(?:大于等于|不小于|>=|≥|大于|超过|高于)\s*(\d+)"
    r"\s*(?:件|个|块)?\s*的?\s*(?:去|到|放入|放到|送往|投放到)?\s*([ABCabc])\s*区"
)
_ZONE_LT_RE = re.compile(
    r"(?:数量|数值)\s*(?:小于|少于|不超过|低于|不大于|<=|≤|<)\s*(\d+)"
    r"\s*(?:件|个|块)?\s*的?\s*(?:去|到|放入|放到|送往|投放到)?\s*([ABCabc])\s*区"
)
_ELSE_RE = re.compile(
    r"(?:其他|其余|剩下的|剩下|else)"
    r"\s*(?:颜色|货物|方块)?"
    r"\s*(?:去|到|放入|放到|送往|投放到)?"
    r"\s*([ABCabc])\s*区"
)


@dataclasses.dataclass(frozen=True)
class ZoneRule:
    """One ordered count-to-zone mapping rule."""

    op: str
    value: Optional[int]
    zone: str

    def matches(self, count: int) -> bool:
        """Return whether this rule applies to a count."""
        if self.op == "ge":
            return count >= int(self.value)
        if self.op == "gt":
            return count > int(self.value)
        if self.op == "le":
            return count <= int(self.value)
        if self.op == "lt":
            return count < int(self.value)
        if self.op == "else":
            return True
        raise TaskProtocolError(f"Unknown zone rule op: {self.op}")


@dataclasses.dataclass(frozen=True)
class MappingRules:
    """Variable-to-colour mapping plus ordered count-to-zone rules."""

    color_map: Mapping[str, str]
    zone_rules: Sequence[ZoneRule]

    def zone_for(self, count: int) -> str:
        """Return the first zone rule that applies to a count."""
        for rule in self.zone_rules:
            if rule.matches(count):
                return rule.zone
        raise TaskProtocolError(
            f"No mapping rule matched count {count}"
        )


def _parse_zone_rules(text: str) -> Sequence[ZoneRule]:
    """Collect zone rules in textual order, skipping overlaps."""
    collected = []
    spans = []
    for regex in (_ZONE_GE_RE, _ZONE_LT_RE):
        for match in regex.finditer(text):
            zone = str(match.group(2)).upper()
            op = "ge" if regex is _ZONE_GE_RE else "lt"
            collected.append(
                (match.start(), ZoneRule(op, int(match.group(1)), zone))
            )
            spans.append(match.span())
    for match in _ELSE_RE.finditer(text):
        if any(
            match.start() < end and start < match.end()
            for start, end in spans
        ):
            continue
        collected.append(
            (
                match.start(),
                ZoneRule("else", None, str(match.group(1)).upper()),
            )
        )
        spans.append(match.span())
    collected.sort(key=lambda entry: entry[0])
    return tuple(rule for _, rule in collected)


def parse_mapping_rules_text(text: str) -> MappingRules:
    """Parse the free-text mapping rules announced before the match."""
    if not isinstance(text, str) or not text.strip():
        raise TaskProtocolError("Mapping rules are empty")
    color_map: Dict[str, str] = {}
    for match in _VAR_COLOR_RE.finditer(text):
        color_map[match.group(1)] = _COLOR_NAMES[match.group(2)]
    zone_rules = _parse_zone_rules(text)
    if not color_map:
        raise TaskProtocolError(
            "Mapping rules must map x/y to a colour, e.g. 'x代表红色'"
        )
    if not zone_rules:
        raise TaskProtocolError(
            "Mapping rules must contain at least one zone rule, "
            "e.g. '数量大于等于3的去A区'"
        )
    if not {"x", "y"}.issubset(color_map):
        raise TaskProtocolError(
            "Mapping rules must map both x and y to colours"
        )
    return MappingRules(
        color_map=color_map, zone_rules=zone_rules
    )


def _parse_mapping_rules_json(text: str) -> MappingRules:
    """Parse a structured JSON mapping-rules alternative."""
    payload = json.loads(text)
    if not isinstance(payload, Mapping):
        raise TaskProtocolError("Mapping rules JSON must be an object")
    color_map = payload.get("color_map")
    if not isinstance(color_map, Mapping):
        raise TaskProtocolError("Mapping rules JSON needs color_map")
    normalized = {
        str(variable): str(colour).lower()
        for variable, colour in color_map.items()
    }
    raw_rules = payload.get("zone_rules")
    if not isinstance(raw_rules, Sequence) or isinstance(raw_rules, str):
        raise TaskProtocolError("Mapping rules JSON needs zone_rules")
    zone_rules = []
    for raw in raw_rules:
        if not isinstance(raw, Mapping):
            raise TaskProtocolError("Each zone rule must be an object")
        op = str(raw.get("op", ""))
        value = raw.get("value")
        zone = str(raw.get("zone", "")).upper()
        if op == "else":
            value = None
        elif not isinstance(value, int) or value < 0:
            raise TaskProtocolError(
                "Zone rule value must be a non-negative int"
            )
        zone_rules.append(ZoneRule(op=op, value=value, zone=zone))
    return MappingRules(color_map=normalized, zone_rules=tuple(zone_rules))


def parse_mapping_rules(mapping: str) -> MappingRules:
    """Parse mapping rules from either free text or a JSON object string."""
    stripped = str(mapping).strip()
    if stripped.startswith("{"):
        try:
            return _parse_mapping_rules_json(stripped)
        except (
            json.JSONDecodeError,
            TaskProtocolError,
            TypeError,
            ValueError,
        ):
            # Not JSON after all; fall through to the free-text parser.
            pass
    return parse_mapping_rules_text(stripped)


# --- word-problem arithmetic -------------------------------------------------


def solve_math_problem(question: str) -> Tuple[int, int]:
    """Return (x, y) from a generator-style arithmetic word problem."""
    if not isinstance(question, str) or not question.strip():
        raise TaskProtocolError("Problem text is empty")
    capacity_match = _CAPACITY_RE.search(question)
    if capacity_match is None:
        raise TaskProtocolError("Problem lacks '每个...有N件' capacity clause")
    capacity = int(capacity_match.group(1))
    if capacity <= 0:
        raise TaskProtocolError("Problem capacity must be positive")

    types_match = _ITEM_TYPES_RE.search(question)
    if types_match is None:
        raise TaskProtocolError("Problem lacks '有A、B两种' item clause")
    item_a = types_match.group(1).strip()
    item_b = types_match.group(2).strip()

    var_match = _VAR_DECL_RE.search(question)
    if var_match is not None:
        x_decl = var_match.group(1)
        if item_a in x_decl:
            x_item, y_item = item_a, item_b
        elif item_b in x_decl:
            x_item, y_item = item_b, item_a
        else:
            x_item, y_item = item_a, item_b
    else:
        x_item, y_item = item_a, item_b

    sums = _sum_demands(question, item_a, item_b)
    x_count = int(math.ceil(sums[x_item] / capacity))
    y_count = int(math.ceil(sums[y_item] / capacity))
    if x_count <= 0 and y_count <= 0:
        raise TaskProtocolError("Problem demands produced zero quantities")
    return x_count, y_count


def _sum_demands(
    question: str,
    item_a: str,
    item_b: str,
) -> Dict[str, int]:
    """Sum explicit and referenced demands without losing person clauses."""
    totals = {item_a: 0, item_b: 0}
    people_demands = {}
    items = (item_a, item_b)
    for text in re.split(r"[;；。]", question):
        stripped = text.strip()
        if _CAPACITY_RE.search(stripped) or _VAR_DECL_RE.search(stripped):
            continue
        same = re.fullmatch(
            r"(.+?)(?:说|也)?和(.+?)(?:需求)?一样"
            r"(?:[,，]但(.+)|但是换成(.+))?", stripped
        )
        if same:
            person, reference, change, replacement = same.groups()
            if reference not in people_demands:
                raise TaskProtocolError("Demand references an unknown person")
            amounts = dict(people_demands[reference])
            if replacement:
                if replacement not in amounts:
                    raise TaskProtocolError("Replacement names unknown item")
                total = sum(amounts.values())
                amounts = dict.fromkeys(items, 0)
                amounts[replacement] = total
            if change:
                reduction = re.fullmatch(
                    r"少(\d+|一|二|两|三|四|五)" + _QUANTITY_UNIT
                    + r"(.+)", change,
                )
                if not reduction or reduction.group(2) not in amounts:
                    raise TaskProtocolError("Unsupported relative demand")
                token = reduction.group(1)
                number = int(token) if token.isdigit() else {
                    "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
                }[token]
                amounts[reduction.group(2)] -= number
                if min(amounts.values()) < 0:
                    raise TaskProtocolError("Relative demand became negative")
            people_demands[person] = amounts
            for item in items:
                totals[item] += amounts[item]
            continue
        if "一样" in text and "和" in text:
            raise TaskProtocolError("Unsupported referenced demand clause")
        demands = list(_DEMAND_RE.finditer(stripped))
        if not demands:
            if (
                re.search(r"\d", stripped)
                and _ITEM_TYPES_RE.search(stripped) is None
            ):
                raise TaskProtocolError(
                    "Unsupported numeric clause: " + stripped
                )
            continue
        subjects = _parse_direct_subjects(stripped[:demands[0].start()])
        amounts = dict.fromkeys(items, 0)
        for match in demands:
            name = match.group(2).strip()
            if name not in amounts:
                raise TaskProtocolError("Demand names an unknown item")
            amounts[name] += int(match.group(1))
        if "双倍" in text:
            tail = text.split("双倍", 1)[1]
            affected = [item for item in items if item in tail] or list(items)
            for item in affected:
                amounts[item] *= 2
        elif "双胞胎" in text:
            raise TaskProtocolError("Twin demand lacks a defined multiplier")
        if not any(amounts.values()):
            raise TaskProtocolError("Demand clause has no explicit quantity")
        for person in subjects:
            people_demands[person] = dict(amounts)
        for item in items:
            totals[item] += amounts[item] * len(subjects)
    return totals


def _parse_direct_subjects(prefix: str) -> Sequence[str]:
    """Extract people without making demand-action vocabulary authoritative."""
    normalized = prefix.strip()
    known = _KNOWN_SUBJECT_PREFIX_RE.fullmatch(normalized)
    if known:
        subject_text = known.group(1).strip().removesuffix("都")
    else:
        generated = _GENERATOR_SUBJECT_PREFIX_RE.fullmatch(normalized)
        if generated is None or not generated.group("action").strip():
            raise TaskProtocolError(
                "Demand clause has no recognizable subject/action structure"
            )
        subject_text = generated.group("subjects")
    subjects = [
        subject.strip()
        for subject in re.split(r"[、，,]|和|与", subject_text)
        if subject.strip()
    ]
    if not subjects:
        raise TaskProtocolError("Demand clause has no subject")
    return tuple(subjects)


def problem_to_plan(
    question: str,
    mapping_rules: str,
    source: str = "rules",
    task_id: Optional[str] = None,
) -> TaskPlan:
    """Solve a generator problem and apply mapping rules into a TaskPlan."""
    x_count, y_count = solve_math_problem(question)
    provincial_items = _provincial_mapping_items(
        x_count, y_count, mapping_rules
    )
    if provincial_items is not None:
        return TaskPlan(
            task_id=task_id or make_task_id(question + mapping_rules),
            source=source,
            items=provincial_items,
        )
    rules = parse_mapping_rules(mapping_rules)
    items = []
    for variable, count in (("x", x_count), ("y", y_count)):
        if count <= 0:
            continue
        colour = rules.color_map.get(variable)
        if colour is None:
            raise TaskProtocolError(
                f"Mapping rules omit variable {variable}"
            )
        zone = rules.zone_for(count)
        items.append(
            TaskItem(color=colour, count=count, target_zone=zone)
        )
    if not items:
        raise TaskProtocolError("Solved problem produced no task items")
    resolved_id = task_id or make_task_id(question + mapping_rules)
    return TaskPlan(
        task_id=resolved_id,
        source=source,
        items=tuple(items),
    )


def _provincial_mapping_items(x_count, y_count, text):
    """Apply the two published value-dependent provincial mappings exactly."""
    # Only normalize presentation; do not guess new thresholds or destinations.
    normalized = re.sub(r"[\s,，;；。:：\"“”'‘’]", "", text).lower()
    threshold_rule = (
        "xy中数值大于等于3值的代表红色否则代表蓝色x去b区y去a区"
    )
    relative_rule = (
        "蓝色去b区红色去c区xy中数值大的是蓝色数值少的是红色"
    )
    if normalized == threshold_rule:
        return tuple(
            TaskItem(
                color="red" if count >= 3 else "blue",
                count=count,
                target_zone=zone,
            )
            for count, zone in ((x_count, "B"), (y_count, "A"))
            if count > 0
        )
    if normalized == relative_rule:
        if x_count == y_count:
            raise TaskProtocolError(
                "Published larger/smaller mapping does not define equal x/y"
            )
        larger = max(x_count, y_count)
        return tuple(
            TaskItem(
                color="blue" if count == larger else "red",
                count=count,
                target_zone="B" if count == larger else "C",
            )
            for count in (x_count, y_count)
            if count > 0
        )
    return None


def validate_problem_calculation(question, calculation):
    """Check model arithmetic before discarding its optional trace."""
    required = {"people", "capacity", "x_total", "y_total", "x", "y"}
    if not isinstance(calculation, Mapping) or set(calculation) != required:
        raise TaskProtocolError("Invalid model calculation fields")
    for key in required - {"people"}:
        value = calculation[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise TaskProtocolError("Model calculation needs nonnegative ints")
    people = calculation["people"]
    if not isinstance(people, list) or not people:
        raise TaskProtocolError("Model calculation needs person rows")
    for person in people:
        if not isinstance(person, Mapping) or set(person) != {
            "name", "x_goods", "y_goods"
        }:
            raise TaskProtocolError("Invalid model person fields")
        if not isinstance(person["name"], str) or not person["name"].strip():
            raise TaskProtocolError("Model person needs a name")
        for key in ("x_goods", "y_goods"):
            value = person[key]
            if isinstance(value, bool) or not isinstance(value, int):
                raise TaskProtocolError("Model person quantity needs an int")
            if value < 0:
                raise TaskProtocolError("Model person quantity is negative")
    capacity = int(_CAPACITY_RE.search(question).group(1))
    if calculation["capacity"] != capacity:
        raise TaskProtocolError("Model capacity disagrees with the question")
    for variable in ("x", "y"):
        total = sum(person[variable + "_goods"] for person in people)
        if total != calculation[variable + "_total"]:
            raise TaskProtocolError("Model demand sum is inconsistent")
        if calculation[variable] != math.ceil(total / capacity):
            raise TaskProtocolError("Model container rounding is inconsistent")
    if (calculation["x"], calculation["y"]) != solve_math_problem(question):
        raise TaskProtocolError("Model arithmetic disagrees with the question")


def is_generator_problem(question: str) -> bool:
    """Return True when the text looks like a generator word problem."""
    return bool(
        _CAPACITY_RE.search(question)
        and _ITEM_TYPES_RE.search(question)
        and _VAR_DECL_RE.search(question)
    )
