import argparse
import datetime
import os
import pathlib
import re
import signal
import subprocess
import time

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
    k2 = "ROBOTIKA_KLOUBAK_SENSOR_CONFIG_2",
    r2 = "EXPLORER_R2_SENSOR_CONFIG_2",
    x2 = "ROBOTIKA_X2_SENSOR_CONFIG_1",
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


def _create_docker(client, name, image, command, mounts=[], environment={}):
    opts = dict(
        name = name,
        command = command,
        # https://github.com/docker/docker-py/pull/2471
        device_requests=[
            docker.types.DeviceRequest(count=-1, capabilities=[['gpu']])
        ],
        privileged = True,
        #network_mode = "host",
        environment = {
            "DISPLAY": os.environ["DISPLAY"],
            "QT_X11_NO_MITSHM": 1,
            "XAUTHORITY": str(XAUTH),
        },
        #stdout = True,
        #stderr = True,
        #security_opt = ["seccomp=unconfined"],
        stdin_open = True, # docker -i
        tty = True,        # docker -t
        detach = True,     # docker -d
        #entrypoint="/bin/bash",
        #command = "/opt/ros/melodic/bin/rviz",
        mounts = [
            docker.types.Mount(str(XAUTH), str(XAUTH), "bind"),
            docker.types.Mount("/tmp/.X11-unix/", "/tmp/.X11-unix/", "bind"),
            docker.types.Mount("/etc/localtime", "/etc/localtime", "bind", read_only=True),
            docker.types.Mount("/dev/input/", "/dev/input/", "bind"),
        ],
    )
    opts["mounts"] += mounts
    opts["environment"].update(environment)
    return client.containers.create(image, **opts)


def _run_sim(client, circuit, logdir, world, robots):
    print("Creating/attaching 'sim-net'")
    try:
        simnet = client.networks.get("simnet")
    except docker.errors.NotFound:
        ipam_pool = docker.types.IPAMPool(
            subnet='172.28.0.0/16',
        )
        ipam_config = docker.types.IPAMConfig(
            pool_configs=[ipam_pool]
        )
        simnet = client.networks.create("simnet", ipam=ipam_config)

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
    environment = dict(
        IGN_PARTITION="sim",
        IGN_IP="172.28.1.1",
    )
    for n, (name, kind) in enumerate(robots.items(), start=1):
        command += [
            f"robotName{n}:={name}",
            f"robotConfig{n}:={kind}"
        ]
    sim = _create_docker(client, "sim", "osrf/subt-virtual-testbed:cloudsim_sim_latest", command, mounts, environment)
    print(f"  connecting to simnet as 172.28.1.1")
    simnet.connect(sim, ipv4_address="172.28.1.1")
    sim.start()
    return sim


def _run_bridge(client, circuit, world, name, kind, n):
    relay_name = f"relay{n}net"
    print(f"Creating/attaching '{relay_name}'")
    try:
        relaynet = client.networks.get(relay_name)
    except docker.errors.NotFound:
        ipam_pool = docker.types.IPAMPool(
            subnet=f'172.{28+n}.0.0/16',
        )
        ipam_config = docker.types.IPAMConfig(
            pool_configs=[ipam_pool]
        )
        relaynet = client.networks.create(f"relay{n}net", ipam=ipam_config)

    bridge_name = f"{name}-bridge"
    print(f"Starting `{bridge_name}` ({kind}) container...")
    command = [
        f"circuit:={circuit}",
        f"worldName:={world}",
        f"robotName1:={name}",
        f"robotConfig1:={kind}"
    ]
    environment = dict(
        IGN_PARTITION="sim",
        IGN_IP=f"172.28.1.{n+1}",
        ROS_MASTER_URI=f"http://172.{28+n}.1.1:11311"
    )
    bridge = _create_docker(client, bridge_name, "osrf/subt-virtual-testbed:cloudsim_bridge_latest", command, environment=environment)
    simnet = client.networks.get("simnet")
    print(f"  connecting to simnet as 172.28.1.{n+1}")
    simnet.connect(bridge, ipv4_address=f"172.28.1.{n+1}")
    print(f"  connecting to {relay_name} as 172.{28+n}.1.1")
    relaynet.connect(bridge, ipv4_address=f"172.{28+n}.1.1")
    bridge.start()
    return bridge

def _run_robot(client, logdir, image, name, n):
    relay_name = f"relay{n}net"
    print(f"Starting `{name}` container...")
    host_file = str(logdir/f"{name}.log")
    container_file = f"/osgar-ws/logs/{name}.log"
    open(host_file, "w").close()
    command = [ "/osgar-ws/run_solution.bash", container_file ]
    mounts = [
        docker.types.Mount(container_file, host_file, "bind"),
    ]
    environment = dict(
        ROS_MASTER_URI=f"http://172.{28+n}.1.1:11311"
    )
    robot = _create_docker(client, name, image, command, mounts, environment)
    relaynet = client.networks.get(relay_name)
    print(f"  connecting to {relay_name} as 172.{28+1}.1.2")
    relaynet.connect(robot, ipv4_address=f"172.{28+n}.1.2")
    robot.start()
    return robot



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


def main(argv=None):
    parser = argparse.ArgumentParser(description='Submit cloudsim run')
    parser.add_argument("config", nargs="?", default=str(pathlib.Path('./run.toml')))
    parser.add_argument("-n", action="store_true", help="dry run")
    args = parser.parse_args(argv)

    config_file = pathlib.Path(args.config).resolve()
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
    symlink = config_file.with_name(config_file.stem)
    if symlink.exists():
        symlink.unlink()
    symlink.symlink_to(f"{strnow}-{config_file.stem}")

    should_stop = False
    def _sigint(signum, frame):
        nonlocal should_stop
        print("got signal, stopping")
        should_stop = True
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    signal.signal(signal.SIGINT, _sigint)

    sim = _run_sim(client, circuit, logdir, world, robots)
    to_stop = [sim]
    stdout = [sim]
    to_wait = []
    for n, (name, kind) in enumerate(robots.items(), start=1):
        if not should_stop:
            b = _run_bridge(client, circuit, world, name, kind, n)
            to_stop.append(b)
            stdout.append(b)
        if not should_stop:
            r = _run_robot(client, logdir, image, name, n)
            to_wait.append(r)
            stdout.append(r)

    if not should_stop:
        print("Waiting for robot containers to finish....")
        while not should_stop and len(to_wait) > 0:
            time.sleep(1)
            for r in to_wait:
                r.reload()
            for r in to_wait:
                if r.status == "exited":
                    print(f"Container {r.name} exited.")
            to_wait = [r for r in to_wait if r.status != "exited"]
            for s in to_stop:
                s.reload()
                if s.status == "exited":
                    print(f"Container {s.name} unexpectedly exited.")
                    break
            else:
                continue
            break

    if len(to_wait) > 0:
        print("Stopping robot containers...")
        for r in to_wait: r.kill(signal.SIGINT) # robot containers respond to signals
        for r in to_wait: r.wait()

    for s in to_stop:
        try:
            s.reload()
            if s.status == "exited":
                print(f"Container {s.name} exited.")
            else:
                print(f"Stopping {s.name} container...")
            #s.kill(signal.SIGINT) # not working because ign containers don't forward signals
            s.stop(timeout=0) # there is no point in waiting since nobody is listening for signals
        except docker.errors.APIError as e:
            print(e)

    for s in stdout:
        with open(logdir / f"{s.name}-stdout.txt", "wb") as f:
            f.write(s.logs())
        s.remove()

    if should_stop:
        # [How to be a proper program](https://www.cons.org/cracauer/sigint.html)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        os.killpg(os.getpid(), signal.SIGINT) # signal.raise_signal(signal.SIGINT) available only since python 3.8



if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
