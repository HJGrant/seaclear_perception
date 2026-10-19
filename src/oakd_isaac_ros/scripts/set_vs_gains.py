#!/usr/bin/env python3
"""Switch the visual-servo staged-PID xy gains, velocity caps and slew limits
in object_detection_ibvs.yaml between known presets.

Only the listed keys are rewritten, in place -- the rest of the file
(including comments) is left untouched. Edits both the source config and
the installed copy (the installed one is a plain copy, not a symlink, so
without this a relaunch would still read the old values). The controller
only reads these at startup: restart the visual-servoing launch afterwards.

Presets (values from the config's git history):
  dubrovnik      ef76c36/fa46e18 -- the historical "Dubrovnik" gains, no slew limit
  physical_1mps  86f7873 -- c4aac88's "real physical values" tuning (~0.6 m/s command)
  scaled_10mps   physical_1mps x16.7 (~10 m/s command), capped at 10 m/s

Usage (on the Orin, inside the container -- the installed copy is root-owned):
    python3 set_vs_gains.py                 # -> dubrovnik
    python3 set_vs_gains.py physical_1mps
    python3 set_vs_gains.py --list
    python3 set_vs_gains.py scaled_10mps --dry-run
"""
import argparse
import re
import sys
from pathlib import Path

PRESETS = {
    'dubrovnik': {
        'staged_pid_xy_kp': 0.05,
        'staged_pid_xy_ki': 0.0,
        'staged_pid_xy_kd': 0.0008,
        'max_vx': 20.0,
        'max_vy': 20.0,
        'slew_vx': 0.0,
        'slew_vy': 0.0,
    },
    'physical_1mps': {
        'staged_pid_xy_kp': 0.0025,
        'staged_pid_xy_ki': 0.0,
        'staged_pid_xy_kd': 0.0,
        'max_vx': 20.0,
        'max_vy': 20.0,
        'slew_vx': 0.5,
        'slew_vy': 0.5,
    },
    'scaled_10mps': {
        'staged_pid_xy_kp': 0.0417,
        'staged_pid_xy_ki': 0.0,
        'staged_pid_xy_kd': 0.0,
        'max_vx': 10.0,
        'max_vy': 10.0,
        'slew_vx': 8.3,
        'slew_vy': 8.3,
    },
}

SRC_CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'oak_d' / 'object_detection_ibvs.yaml'


def default_targets():
    targets = [SRC_CONFIG]
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('oakd_isaac_ros')) / 'config' / 'oak_d' / 'object_detection_ibvs.yaml'
        if installed.exists() and installed.resolve() != SRC_CONFIG.resolve():
            targets.append(installed)
    except Exception:
        print('Note: oakd_isaac_ros install not found (workspace not sourced?) -- '
              'only editing the source config.')
    return targets


def apply(path: Path, name: str, values: dict, dry_run: bool) -> bool:
    text = path.read_text()
    for key, value in values.items():
        pattern = re.compile(rf'^(?P<indent>[ \t]*){re.escape(key)}:[^\n]*$', re.MULTILINE)
        matches = pattern.findall(text)
        if len(matches) != 1:
            print(f'  ERROR: expected exactly one "{key}:" line in {path}, found {len(matches)}')
            return False
        old_line = pattern.search(text).group(0)
        new_line = f'{pattern.search(text).group("indent")}{key}: {value} # preset: {name}'
        if old_line.strip() != new_line.strip():
            print(f'  {key:18} {old_line.split(":", 1)[1].strip():50} -> {value}')
        text = pattern.sub(lambda m: f'{m.group("indent")}{key}: {value} # preset: {name}', text)

    try:
        import yaml
        params = yaml.safe_load(text)['visual_servo_controller_node']['ros__parameters']
        bad = [k for k, v in values.items() if params.get(k) != v]
        if bad:
            print(f'  ERROR: re-parsed values do not match for {bad}; not writing {path}')
            return False
    except ImportError:
        pass

    if not dry_run:
        path.write_text(text)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('preset', nargs='?', default='dubrovnik', choices=sorted(PRESETS))
    parser.add_argument('--file', type=Path, action='append',
                        help='Config file(s) to edit (default: source + installed copy)')
    parser.add_argument('--dry-run', action='store_true', help='Show the changes without writing')
    parser.add_argument('--list', action='store_true', help='Print the presets and exit')
    args = parser.parse_args()

    if args.list:
        for name, values in PRESETS.items():
            print(f'{name}:')
            for k, v in values.items():
                print(f'  {k}: {v}')
        return

    ok = True
    for path in args.file or default_targets():
        print(f'{"[dry run] " if args.dry_run else ""}{path}  -> preset "{args.preset}"')
        ok &= apply(path, args.preset, PRESETS[args.preset], args.dry_run)
    if ok and not args.dry_run:
        print('Done. Restart the visual-servoing launch to load the new values.')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
