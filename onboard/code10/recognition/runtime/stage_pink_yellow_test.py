#!/usr/bin/env python3
"""Stage the local pink-then-yellow balloon test without ASR, a model, or HTTP."""

from __future__ import annotations

import json

from uav_natural_language import process_structured_plan


def build_plan() -> dict:
    return {
        "version": 2,
        "actions": [
            {
                "capability": "strike_balloon",
                "parameters": {"color": "pink"},
            },
            {
                "capability": "strike_balloon",
                "parameters": {"color": "yellow"},
            },
        ],
    }


def main() -> int:
    accepted = process_structured_plan(build_plan())
    print(json.dumps(accepted, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
