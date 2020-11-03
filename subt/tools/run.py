import argparse
import datetime
import os
import pathlib
import re
import signal
import subprocess

import docker
import docker.errors
import docker.types
import toml

from pprint import pprint

WORLDS=dict(
    #TUNNEL
    tq  = "tunnel_qual_ign",
    ts1 = "simple_tunnel_01",
    ts2 = "simple_tunnel_02",
    ts3 = "simple_tunnel_03",
    tp1 = "tunnel_circuit_practice_01",
    tp2 = "tunnel_circuit_practice_02",
    tp3 = "tunnel_circuit_practice_03",
    tc1 = "tunnel_circuit_01",
    tc7 = "tunnel_circuit_07",
    tc12 = "tunnel_circuit_12",
    tc13 = "tunnel_circuit_13",
    tc15 = "tunnel_circuit_15",

    # URBAN
    uq  = "urban_qual",
    us1 = "simple_urban_01",
    us2 = "simple_urban_02",
    us3 = "simple_urban_03",
    up1 = "urban_circuit_practice_01",
    up2 = "urban_circuit_practice_02",
    up3 = "urban_circuit_practice_03",
    uc1 = "urban_circuit_01",
    uc2 = "urban_circuit_02",
    uc3 = "urban_circuit_03",
    uc4 = "urban_circuit_04",
    uc5 = "urban_circuit_05",
    uc6 = "urban_circuit_06",
    uc7 = "urban_circuit_07",
    uc8 = "urban_circuit_08",

    #CAVE
    cq  = "cave_qual",
    cs1 = "simple_cave_01",
    cs2 = "simple_cave_02",
    cs3 = "simple_cave_03",
    cp1 = "cave_circuit_practice_01",
    cp2 = "cave_circuit_practice_02",
    cp3 = "cave_circuit_practice_03",
)

ROBOTS=dict(
    teambase = "TEAMBASE",
    drone = "SSCI_X4_SENSOR_CONFIG_2",
    freyja = "ROBOTIKA_FREYJA_SENSOR_CONFIG_2",
)

XAUTH = pathlib.Path("/tmp/.docker.xauth")


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
    #valid[f"T{timeout}"] = "TEAMBASE"
    return valid


def validate_image(client, image):
    image_name = f"robotika:{image}"
    try:
        image = client.images.get(image_name)
        return image_name
    except docker.errors.ImageNotFound:
        sys.exit(f"image '{image_name}' not found")


def validate_circuit(world):
    for c in "tunnel", "urban", "cave":
        if c in world:
            return c
    sys.exit(f"autodetection of circuit failed for world {world}")


def _run_docker(client, name, image, command, mounts={}):
    opts = dict(
        name = name,
        command = command,
        # https://github.com/docker/docker-py/pull/2471
        device_requests=[
            docker.types.DeviceRequest(count=-1, capabilities=[['gpu']])
        ],
        #remove = True,
        privileged = True,
        network_mode = "host",
        environment = {
            "DISPLAY": os.environ["DISPLAY"],
            "QT_X11_NO_MITSHM": 1,
            "XAUTHORITY": str(XAUTH),
        },
        #stdout = True,
        #stderr = True,
        #security_opt = ["seccomp=unconfined"],
        #stdin_open = True,
        #entrypoint="/bin/bash",
        #command = "/opt/ros/melodic/bin/rviz",
        mounts = [
            docker.types.Mount(str(XAUTH), str(XAUTH), "bind"),
            docker.types.Mount("/tmp/.X11-unix/", "/tmp/.X11-unix/", "bind"),
            docker.types.Mount("/etc/localtime", "/etc/localtime", "bind", read_only=True),
            docker.types.Mount("/dev/input/", "/dev/input/", "bind"),
        ],
        detach = True,
    )
    opts["mounts"] += mounts
    return client.containers.run(image, **opts)


def _run_sim(client, circuit, logdir, world, robots):
    print("Starting 'sim' container...")
    command = [
        "cloudsim_sim.ign",
        "headless:=1",
        "seed:=1",
        f"circuit:={circuit}",
        f"worldName:={world}"
    ]
    mounts = [
        docker.types.Mount("/tmp/ign/logs", str(logdir), "bind"),
    ]
    for name, kind in robots.items():
        command += [
            f"robotName1:={name}",
            f"robotConfig1:={kind}"
        ]
    return _run_docker(client, "sim", "osrf/subt-virtual-testbed:cloudsim_sim_latest", command, mounts)


def _run_bridge(client, circuit, world, name, kind):
    bridge_name = f"{name}-bridge"
    print(f"Starting `{bridge_name}` ({kind}) container...")
    command = [
        f"circuit:={circuit}",
        f"worldName:={world}",
        f"robotName1:={name}",
        f"robotConfig1:={kind}"
    ]
    return _run_docker(client, bridge_name, "osrf/subt-virtual-testbed:cloudsim_bridge_latest", command)


def _run_robot(client, logdir, image, name):
    print(f"Starting `{name}` container...")
    host_file = str(logdir/f"{name}.log")
    container_file = f"/osgar-ws/logs/{name}.log"
    open(host_file, "w").close()
    command = [ "/osgar-ws/run_solution.bash", container_file ]
    mounts = [
        docker.types.Mount(container_file, host_file, "bind"),
    ]
    return _run_docker(client, name, image, command, mounts)


def _cmd(*args, input=None):
    return subprocess.run(args, stdout=subprocess.PIPE, input=input).stdout


def _xauth():
    """Make sure processes in the container can connect to the x server
    "Necessary so gazebo can create a context for OpenGL rendering (even headless)"""
    if not XAUTH.exists():
        DISPLAY = os.getenv("DISPLAY")
        xauth_list = _cmd("xauth", "nlist", DISPLAY)
        xauth_list = re.sub(b"^....", b"ffff", xauth_list)
        #print(str(xauth_list, encoding='ascii'))
        if xauth_list:
            _cmd("xauth", "-f", XAUTH, "nmerge", "-", input=xauth_list)
        else:
            _cmd("touch", XAUTH)
        XAUTH.chmod(0o0644)


def _prune_containers(client, robots):
    container_names = {'sim'} | set(robots.keys()) | { f"{name}-bridge" for name in robots.keys() }
    #print(sorted(container_names))
    for c in client.containers.list(all=True):
        if c.name in container_names:
            try:
                c.remove()
            except docker.errors.APIError as e:
                print(c.name, file=sys.stderr)
                sys.exit(str(e))


def main(argv):
    parser = argparse.ArgumentParser(description='Submit cloudsim run')
    parser.add_argument("config", nargs="?", default=str(pathlib.Path('./run.toml')))
    parser.add_argument("-n", action="store_true", help="dry run")
    args = parser.parse_args(argv)

    config_file = pathlib.Path(args.config).absolute()
    if not config_file.is_file():
        sys.exit(f"{config_file} not found")

    with open(args.config, 'r') as f:
        config = toml.load(f)

    client = docker.from_env()
    version = client.version()["Version"]
    if version < "19.03":
        sys.exit("Please install at least version 19.03 of docker")

    world = validate_world(config["world"])
    circuit = validate_circuit(world)
    robots = validate_robots(config["robots"], config["timeout"])
    image = validate_image(client, config["image"])
    now = datetime.datetime.now(datetime.timezone.utc)
    strnow = f"{now.year}-{now.month:02d}-{now.day:02d}T{now.hour:02d}.{now.minute:02d}.{now.second:02d}"
    logdir = config_file.with_name(f"{strnow}-{config_file.stem}")

    print(f"circuit: {circuit}")
    print(f"world:   {world}")
    print(f"image:   {image}")
    print(f"logdir:  {logdir}")
    print(f"robots:  ")
    for name, kind in robots.items():
        print(f"  {name:>10s}: {kind}")
    print()

    _prune_containers(client, robots)
    _xauth()
    logdir.mkdir()

    should_stop = False
    def _sigint(signum, frame):
        nonlocal should_stop
        print("got signal")
        should_stop = True
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    signal.signal(signal.SIGINT, _sigint)

    sim = _run_sim(client, circuit, logdir, world, robots)
    to_stop = [sim]
    to_wait = []
    for name, kind in robots.items():
        if not should_stop:
            b = _run_bridge(client, circuit, world, name, kind)
            to_stop.append(b)
        if not should_stop:
            r = _run_robot(client, logdir, image, name)
            to_wait.append(r)

    if not should_stop:
        print("Waiting for robot containers to finish....")
        import time
        while not should_stop and len(to_wait) > 0:
            time.sleep(1)
            print(f"shoud_stop: {should_stop}", f"len(to_wait): {len(to_wait)}")
            for r in to_wait:
                r.reload()
            to_wait = [r for r in to_wait if r.status != "exited"]

    for r in to_wait: r.kill(signal.SIGINT) # robot containers respond to signals
    for r in to_wait: r.wait()

    for s in to_wait + to_stop:
        try:
            s.reload()
            if s.status == "exited":
                print(f"Container {s.name} exited.")
            else:
                print(f"Stopping {s.name} container...")
            #s.kill(signal.SIGINT) # not working because ign containers don't forward signals
            s.stop()
        except docker.errors.APIError as e:
            print(e)


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
