#!/usr/bin/env python
"""
Main script
- program: -t <type> -s <serial[s]>
- ui: ...
"""

import argparse
import sys

from . import ui
from . import utils


parser = argparse.ArgumentParser(
    description="go stompy go!")

parser.add_argument(
    "command", type=str, choices=["program", "ui", "reset"])
parser.add_argument("-t", "--type", type=str, default=None)
#parser.add_argument("-s", "--serials", type=str, default=None)

args = parser.parse_args(sys.argv[1:])

if args.command == 'ui':
    # start ui
    print("Starting ui")
    ui.start()
elif args.command == 'program':
    # program teensies
    if args.type is not None:
        types = args.type.split(',')
    else:
        types = None
    print("Programming teensies...")
    utils.program_teensies_by_type(types)
elif args.command == 'reset':
    # reset teensies
    if args.type is not None:
        types = args.type.split(',')
    else:
        types = None
    print("Resetting teensies...")
    utils.reset_teensies_by_type(types)
