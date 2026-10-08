#!/usr/bin/env python3
"""Format/check C++ with the unchanged ROS Rolling ament configuration."""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Fail on formatting divergence (default)')
    mode.add_argument('--fix', action='store_true', help='Apply formatting and verify the result')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    try:
        import ament_clang_format.main as formatter
    except ImportError:
        print('Install developer tools: python -m pip install -r tools/requirements-format.txt',
              file=sys.stderr)
        return 1

    executable = shutil.which('clang-format')
    if not executable:
        print('clang-format 21.1.8 is required; install tools/requirements-format.txt', file=sys.stderr)
        return 1
    version = subprocess.check_output([executable, '--version'], text=True)
    if not re.search(r'\bversion 21\.1\.8\b', version):
        print('Expected clang-format 21.1.8, got: ' + version.strip(), file=sys.stderr)
        return 1

    config = root / '.clang-format'
    official = Path(formatter.__file__).parent / 'configuration' / '.clang-format'
    if config.read_bytes() != official.read_bytes():
        print('The repository config must match the pinned Rolling ament config byte-for-byte.',
              file=sys.stderr)
        return 1

    files = sorted(
        str(path) for directory in ('include', 'src', 'tests', 'benchmarks')
        for path in (root / directory).rglob('*')
        if path.is_file() and path.suffix in ('.cpp', '.hpp'))
    command = ['--config', str(config), *files]
    if args.fix:
        # ament reports divergence on the first pass even after applying it.
        # The final official check decides success and verifies idempotence.
        formatter.main(['--reformat', *command])
    return formatter.main(command)


if __name__ == '__main__':
    sys.exit(main())
