#!/usr/bin/env python3
"""Check all 48 expected tags using latest evidence, never just markers."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from summarize48_v2 import rows_for  # noqa: E402  # noqa: E402


def main():
    """Require complete coverage and the current 290-second target."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    rows = rows_for(args.root)
    bad = [r for r in rows if not r['under290']
           or r['requirements_verified'] is not True or r['goods_score'] != 20]
    print(json.dumps({'expected': 48, 'rows': len(rows),
                      'accepted': len(rows) - len(bad),
                      'failing': [r['tag'] for r in bad]}, indent=2))
    return 0 if len(rows) == 48 and not bad else 1


if __name__ == '__main__':
    sys.exit(main())
