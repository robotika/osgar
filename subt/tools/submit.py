import argparse
import json
import re
import requests
import subprocess
import os
import pathlib

import toml

# See https://pkg.go.dev/gitlab.com/ignitionrobotics/web/cloudsim/simulations for original names
WORLDS = dict(
    tq  = "Tunnel Qualification",
    tp1 = "Tunnel Practice 1",
    tp2 = "Tunnel Practice 2",
    tp3 = "Tunnel Practice 3",
    tc1 = "Tunnel Circuit World 1",
    tc2 = "Tunnel Circuit World 2",
    tc3 = "Tunnel Circuit World 3",
    tc4 = "Tunnel Circuit World 4",
    tc5 = "Tunnel Circuit World 5",
    tc6 = "Tunnel Circuit World 6",
    tc7 = "Tunnel Circuit World 7",
    tc8 = "Tunnel Circuit World 8",

    uq  = "Urban Qualification",
    up1 = "Urban Practice 1",
    up2 = "Urban Practice 2",
    up3 = "Urban Practice 3",
    uc1 = "Urban Circuit World 1",
    uc2 = "Urban Circuit World 2",
    uc3 = "Urban Circuit World 3",
    uc4 = "Urban Circuit World 4",
    uc5 = "Urban Circuit World 5",
    uc6 = "Urban Circuit World 6",
    uc7 = "Urban Circuit World 7",
    uc8 = "Urban Circuit World 8",

    cq  = "Cave Qualification",
    cs1 = "Cave Simple 1",
    cs2 = "Cave Simple 2",
    cs3 = "Cave Simple 3",
    cp1 = "Cave Practice 1",
    cp2 = "Cave Practice 2",
    cp3 = "Cave Practice 3",
    cc  = "Cave Circuit",
    cc1 = "Cave Circuit World 1",
    cc2 = "Cave Circuit World 2",
    cc3 = "Cave Circuit World 3",
    cc4 = "Cave Circuit World 4",
    cc5 = "Cave Circuit World 5",
    cc6 = "Cave Circuit World 6",
    cc7 = "Cave Circuit World 7",
    cc8 = "Cave Circuit World 8",

    fq = "Finals Qualification",
)

ROBOTS=dict(
    teambase = "TEAMBASE",
    drone = "SSCI_X4_SENSOR_CONFIG_2",
    freyja = "ROBOTIKA_FREYJA_SENSOR_CONFIG_2",
    k2 = "ROBOTIKA_KLOUBAK_SENSOR_CONFIG_2",
    r2 = "EXPLORER_R2_SENSOR_CONFIG_2",
)


def _cmd(*args):
    return subprocess.run(args, stdout=subprocess.PIPE).stdout


def validate_world(world):
    try:
        return WORLDS[world]
    except KeyError:
        sys.exit(f"'{world}' not valid world identification")


def validate_robots(robots, timeout):
    valid = {}
    for name, kind in robots.items():
        try:
            valid[name] = ROBOTS[kind]
        except KeyError:
            sys.exit(f"'{kind}' not valid robot kind identification")
    valid[f"T{timeout}"] = "TEAMBASE"
    return valid


def validate_image(image):
    images = _cmd('aws', 'ecr', 'list-images', '--repository-name', 'subt/robotika', '--output', 'json')
    for img in json.loads(images)["imageIds"]:
        if img.get('imageTag') == image:
            return f'138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/robotika:{image}'
    sys.exit(f"image '{image}' not found in aws subt/robotika repository")


def generate_simname(logdir, image, world):
    if not logdir.exists():
        sys.exit(f"{logdir} does not exist")
    image = image.replace('-', '').replace('_', '')
    prefix = image+world
    existing = [x for x in logdir.iterdir() if x.is_dir() and x.name.startswith(prefix)]
    run = 0
    for a in sorted(existing):
        run = re.search('(?<=[vr])[0-9]+$', a.name).group()
    run = int(run) + 1
    simname = f"{prefix}r{run}"
    return simname


def _pretty_print_request(req):
    print('{}\n{}\r\n{}\r\n\r\n{}'.format(
        '-----------START-----------',
        req.method + ' ' + req.url,
        '\r\n'.join('{}: {}'.format(k, v) for k, v in req.headers.items()),
        req.body.decode('utf-8'),
    ))


def submit(token, simname, image, world, robots, dont_submit):
    data = [
        ('name', (None, simname)),
        ('owner', (None, "robotika")),
        ('circuit', (None, world)),
    ]
    for name, kind in robots.items():
        data.append(('robot_name', (None, name)))
        data.append(('robot_type', (None, kind)))
        data.append(('robot_image', (None, image)))

    headers = {
        'Private-Token': token
    }

    req = requests.Request('POST', url="https://cloudsim.ignitionrobotics.org/1.0/simulations", files=data,
                           headers=headers).prepare()

    if dont_submit:
        print("======= not really submiting =======")
        _pretty_print_request(req)
        return False

    s = requests.Session()
    resp = s.send(req)

    print(json.dumps(resp.json(), indent=4))

    if resp.status_code == 200:
        return True

    return False


def main(argv):
    token = os.getenv("SUBT_ACCESS_TOKEN")
    if token is None:
        sys.exit("SUBT_ACCESS_TOKEN environment variable not set\n"
                 "https://subtchallenge.world/settings -> Access Tokens -> Create Token\n"
                 "assing the token to the SUBT_ACCESS_TOKEN variable")

    parser = argparse.ArgumentParser(description='Submit cloudsim run')
    parser.add_argument("config", nargs="?", default=str(pathlib.Path('./run.toml')))
    parser.add_argument("-n", action="store_true", help="don't really submit")
    args = parser.parse_args(argv)

    config_file = pathlib.Path(args.config).absolute()
    if not config_file.is_file():
        sys.exit(f"{config_file} not found")

    with open(args.config, 'r') as f:
        config = toml.load(f)

    world = validate_world(config["world"])
    robots = validate_robots(config["robots"], config["timeout"])
    image_url = validate_image(config["image"])
    simname = generate_simname(config_file.parent, config["image"], config["world"])

    print(f"world:   {world}")
    print(f"image:   {image_url}")
    print(f"simname: {simname}")
    print(f"robots:  ")
    for name, kind in robots.items():
        print(f"  {name:>10s}: {kind}")

    if submit(token, simname, image_url, world, robots, args.n):
        (config_file.parent / simname).mkdir()


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
