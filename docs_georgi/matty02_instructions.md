# Activation and shutdown

1. `cd ~/git/osgar`

2. `source /home/robot/git/matty/odroid/python/.venv/bin/activate`

3. `python -m osgar.record ./config/matty-go.json --params app.dist=0.3 app.max_speed=0.2` 
* the logs are saved to `/home/robot/logs/`

4. `shutdown now`

# Connect to WiFi

1. connect via LAN cable the O-DROID motherboard

2. `nmcli d`

3. `sudo nmcli r wifi on`

4. `sudo nmcli d wifi connec SSID password PASSWORD`

# Transfer files

Logs location on m02: `/home/robot/logs`

My location for tests: `D:\Gogi\Documents\robotics_general\logs\m02_test`

Use WinSCP... Log-in to the robot... Drag-and-drop

# Play logs

Use my custom log_analyzer.

# Follow me

1. `cd ~/git/osgar-apps/followme`

2. `source /home/robot/git/matty/odroid/python/.venv/bin/activate`

3.  `python -m osgar.record ./config/matty-follow-person-day.json --params app.max_speed=0.1`


# Tailscale VPN (WireGuard protocol)

1. `curl -fsSL https://tailscale.com/install.sh | sh`

2. `sudo systemctl enable --now tailscaled`

3. `sudo tailscale up --authkey=tskey-yourkeyhere`

## In case curl is missing:

1. `sudo apt update`

2. `sudo apt install curl`

## Useful commands

### screen

Even if we loose connection to the robot, we can still keep track of the terminal. The terminal is still on on the background.

Before screen I should activate the virtual environment, and then I should do the same after screen activates.

#### -dr

Forcefully disconnect the other instance and bring the session here. But just from the view, the other side remains logged in via VPN.

#### -x

Share the same view across multiple machines (shared desktop).

#### -ls

See all monitors.

#### -dr SESSION_NUM

Reconnect to a session (monitor).

### who

See all machines that are still logged in (and receive broadcast).

### wall

"Write All": It’s a command-line utility that sends a message to the terminals of every single user currently logged into the system.

# Martin's remote session 1: matty-click2go

1. `cd git/osgar`
2. `git diff`
3. `git branch`
4. `cd ../osgar-apps/`
5. `git branch`
6. `cd ~/git/matty/`
7. `git branch`
8. `source /home/robot/git/matty/odroid/python/.venv/bin/activate`
9. `python`
10. `cd ../osgar`
11. `git checkout feature/zmq-pub-sub`
12. `cd ../osgar-apps/`
13. `git checkout feature/click2go`
14. `cd click2go/`
15. `python -m osgar.record config/matty-click2go.json --note "remotely m2d"`
16. `cd ..`
17. `git checkout feature/depthai_v3`
18. `cd ~/git/osgar`
19. `git checkout feature/depthai3`

# Run module in a loop

The safety button stops the current run and starts waiting. After the button is released, the script is reactivated. The robot also doesn't have to be connected to the terminal to run. Tested and works well.

* `../dtc-systems/run_dtc_loop.sh ~/git/osgar/config/matty-wait-for-start.json config/dtc-night.json`

* `~/git/osgar-apps/dtc-systems/run_dtc_loop.sh ~/git/osgar/config/matty-wait-for-start.json ~/git/osgar-apps/followme/config/matty-follow-person-night.json --params app.max_speed=0.42`