#!/usr/bin/env python3
"""First UAV capability: strike a balloon of a specific or arbitrary color."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


PLAN_VERSION = 2
MAX_ACTIONS = 8
ANY_COLOR = "any"
SUPPORTED_COLORS = ("red", "orange", "yellow", "green", "blue", "purple", "pink")
ALLOWED_COLOR_ARGUMENTS = (ANY_COLOR, *SUPPORTED_COLORS)
COLOR_GROUPS = {
    "yellow": "yellow",
    "green": "yellow",
    "pink": "pink",
    "red": "pink",
    "orange": "pink",
    "blue": "blue",
    "purple": "blue",
}


class CapabilityError(ValueError):
    """Raised when a request is outside the installed capability interface."""


@dataclass(frozen=True)
class CapabilityDefinition:
    name: str
    description: str
    available: bool
    executor: str
    parameters: dict[str, Any]
    completion: str


CAPABILITIES: dict[str, CapabilityDefinition] = {
    "strike_balloon": CapabilityDefinition(
        name="strike_balloon",
        description="撞击指定颜色或任意颜色的气球",
        available=True,
        executor="balloon_target_chase",
        parameters={"color": list(ALLOWED_COLOR_ARGUMENTS)},
        completion="near_then_missing",
    ),
    # Stop is retained as an independent local safety primitive.  It is not a
    # flight skill and never depends on perception or a language model.
    "stop": CapabilityDefinition(
        name="stop",
        description="立即停止当前任务",
        available=True,
        executor="local_stop",
        parameters={},
        completion="immediate",
    ),
}

DEFAULT_TEST_ACTION: dict[str, Any] = {
    "capability": "strike_balloon",
    "parameters": {"color": ANY_COLOR},
}


def capability_catalog() -> dict[str, Any]:
    return {
        "version": PLAN_VERSION,
        "max_actions": MAX_ACTIONS,
        "capabilities": [asdict(item) for item in CAPABILITIES.values()],
        "default_test_action": DEFAULT_TEST_ACTION,
    }


def normalize_legacy_action(raw: dict[str, Any]) -> dict[str, Any]:
    """Convert the previously deployed plan format during the upgrade."""
    if raw.get("type") == "stop":
        return {"capability": "stop", "parameters": {}}
    if raw.get("type") == "strike_color":
        return {
            "capability": "strike_balloon",
            "parameters": {"color": raw.get("color")},
        }
    return raw


def validate_action(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise CapabilityError("each action must be an object")
    raw = normalize_legacy_action(raw)
    if set(raw) - {"capability", "parameters"}:
        raise CapabilityError("action contains unsupported fields")
    name = raw.get("capability")
    if not isinstance(name, str) or name not in CAPABILITIES:
        raise CapabilityError(f"unknown capability: {name!r}")
    parameters = raw.get("parameters", {})
    if not isinstance(parameters, dict):
        raise CapabilityError("parameters must be an object")

    if name == "stop":
        if parameters:
            raise CapabilityError("stop does not accept parameters")
        return {"capability": "stop", "parameters": {}}

    if set(parameters) != {"color"}:
        raise CapabilityError("strike_balloon requires exactly a color parameter")
    color = parameters.get("color")
    if color not in ALLOWED_COLOR_ARGUMENTS:
        raise CapabilityError(f"unsupported balloon color: {color!r}")
    return {"capability": "strike_balloon", "parameters": {"color": str(color)}}


def validate_actions(raw_actions: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_actions, list) or not 1 <= len(raw_actions) <= MAX_ACTIONS:
        raise CapabilityError(f"actions must contain 1-{MAX_ACTIONS} items")
    actions = [validate_action(item) for item in raw_actions]
    if any(item["capability"] == "stop" for item in actions[:-1]):
        raise CapabilityError("stop may only be the final action")
    return actions


def validate_plan(raw_plan: Any) -> dict[str, Any]:
    if not isinstance(raw_plan, dict):
        raise CapabilityError("plan must be an object")
    return {"version": PLAN_VERSION, "actions": validate_actions(raw_plan.get("actions"))}


def canonical_balloon_color(color: Any) -> str:
    """Map a requested or observed color into the three flight color groups."""
    normalized = str(color or "").strip().lower()
    if normalized == ANY_COLOR:
        return ANY_COLOR
    return COLOR_GROUPS.get(normalized, normalized)


def balloon_matches(detection: dict[str, Any], color: str) -> bool:
    if "balloon" not in str(detection.get("class_name", "")).strip().lower():
        return False
    requested_group = canonical_balloon_color(color)
    return (
        requested_group == ANY_COLOR
        or canonical_balloon_color(detection.get("color_name")) == requested_group
    )
