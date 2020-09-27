"""
  Virtual SubT Challenge - decoder of robot name
"""

def parse_robot_name(robot_name):
    options = {
        'L': 'left',
        'R': 'right',
        'C': 'center',
        'W': 'wait'
    }
    times_sec = [int(x) for x in robot_name[1:-1].split('F')]
    side = options[robot_name[-1]]
    ret = []
    ret.append(('wait', times_sec[0]))
    for t in times_sec[1:]:
        ret.append((side, t))
        ret.append(('home', t*2))
    return ret

# vim: expandtab sw=4 ts=4
