#!/usr/bin/env python3

import subprocess
from pathlib import Path

from blessings import Terminal


def main():

    all_robots = {
        "k2": "root@10.0.0.21:/home/robot/git/osgar/logs/",
        "skiddy": "owner@10.0.0.30:/home/owner/Develop/osgar/sync/",
    }

    import argparse
    parser = argparse.ArgumentParser(description='rsync logs from robots')
    parser.add_argument('robots', nargs='*', choices=list(all_robots.keys()) + [[]], default=[])
    args = parser.parse_args()

    if args.robots == []:
        robots = all_robots
    else:
        robots = {k: all_robots[k] for k in args.robots}

    home = str(Path.home())
    t = Terminal()
    try:
        while True:
            for robot, logs in robots.items():
                print(t.bold(f"Syncing {robot}:"))
                print(t.dim, end="", flush=True)
                a = subprocess.call(["rsync", "-v", "--append", "--progress", "--recursive", f"{logs}", f"{home}/logs/{robot}"])
                print(t.normal, end="", flush=True)
                if a == 0:
                    print(t.green("OK"))
                else:
                    print(t.red("Failed"))
                print()
    except KeyboardInterrupt:
        print(t.normal, flush=True)


if __name__ == "__main__":
    main()
