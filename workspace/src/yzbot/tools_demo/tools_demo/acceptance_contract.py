"""Pure helpers for validating the competition ROS graph contract."""

from typing import Any, Dict, Iterable, List, Mapping, Sequence, Set, Tuple


def fully_qualified_node_names(
    entries: Iterable[Sequence[str]],
) -> Set[str]:
    """Convert ROS node-name and namespace pairs to absolute names."""
    names = set()
    for name, namespace in entries:
        prefix = str(namespace).rstrip("/")
        names.add(f"{prefix}/{name}" if prefix else f"/{name}")
    return names


def typed_interface_map(
    entries: Iterable[Tuple[str, Sequence[str]]],
) -> Dict[str, Set[str]]:
    """Normalize graph interface entries for exact type checks."""
    return {
        str(name): {str(interface_type) for interface_type in types}
        for name, types in entries
    }


def missing_typed_interfaces(
    required: Iterable[str],
    available: Dict[str, Set[str]],
) -> List[str]:
    """Return absent or mistyped ``name|type`` interface contracts."""
    missing = []
    for specification in required:
        name, separator, interface_type = str(specification).partition("|")
        if not separator or not name or not interface_type:
            missing.append(f"invalid:{specification}")
            continue
        if interface_type not in available.get(name, set()):
            missing.append(str(specification))
    return missing


def parse_transform_specification(specification: str) -> Tuple[str, str]:
    """Parse one ``target:source`` TF requirement."""
    target, separator, source = str(specification).partition(":")
    if not separator or not target or not source:
        raise ValueError(
            "transform specifications must use target:source"
        )
    return target, source


def validated_llm_plan(status: Mapping[str, Any]) -> bool:
    """Return whether a real model produced the validated active plan."""
    return (
        status.get("available") is True
        and status.get("source") == "llm"
    )


def validated_avoidance(status: Mapping[str, Any]) -> bool:
    """Require two distinct observed responses and collision-free samples."""
    try:
        obstacles_seen = int(status.get("obstacles_seen", 0))
        sample_count = int(status.get("sample_count", 0))
    except (TypeError, ValueError):
        return False
    events = status.get("avoidance_events", [])
    if not isinstance(events, (list, tuple)) or not all(
            isinstance(event, str) and event for event in events):
        return False
    return (
        obstacles_seen >= 2
        and sample_count > 0
        and status.get("safe") is True
        and status.get("collision_detected") is False
        and status.get("avoidance_demonstrated") is True
        and len(set(events)) >= 2
    )


def successful_full_mission(
    mission: Mapping[str, Any],
    *,
    require_llm: bool,
    llm_status: Mapping[str, Any],
    require_avoidance: bool,
    avoidance_status: Mapping[str, Any],
) -> bool:
    """Require every requested cube plus model and avoidance evidence."""
    try:
        completed = int(mission.get("completed", 0))
        total = int(mission.get("total", 0))
        skipped = int(mission.get("skipped", 0))
    except (TypeError, ValueError):
        return False
    return (
        mission.get("state") == "completed"
        and total > 0
        and completed == total
        and skipped == 0
        and (not require_llm or validated_llm_plan(llm_status))
        and (
            not require_avoidance
            or validated_avoidance(avoidance_status)
        )
    )
