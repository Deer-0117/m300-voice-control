#!/usr/bin/env python3
"""Optional capture service. Disabled configurations perform no writes."""
import argparse
import json

from hardcase.service import load_config, run_service


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--check-config', action='store_true', help='validate without starting capture')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        if args.check_config:
            print(json.dumps({'valid': True, 'enabled': config['enabled']}))
            return 0
        return run_service(config)
    except (ValueError, OSError, RuntimeError) as exc:
        parser.exit(2, str(exc)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
