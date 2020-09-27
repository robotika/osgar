"""
  Virtual SubT Challenge - decoder of robot name
"""

def split_multi(s, delimiters):
    # split string s via multiple character delimiters
    ret = []
    word = ''
    for c in s:
        word += c
        if c in delimiters:
            ret.append(word)
            word = ''
    if word != '':
        ret.append(word)
    return ret


def parse_robot_name(robot_name):
    options = {
        'L': 'left',
        'R': 'right',
        'C': 'center',
        'W': 'wait'
    }
    parts = robot_name[1:-1].split('F')
    default_side = options[robot_name[-1]]
    ret = []
    ret.append(('wait', int(parts[0])))
    for run in parts[1:]:
        sum_t = 0
        for part in split_multi(run, options.keys()):
            try:
                t = int(part)
                side = default_side
            except ValueError:
                t = int(part[:-1])
                side = options[part[-1]]
            ret.append((side, t))
            sum_t += t
        ret.append(('home', sum_t * 2))
    return ret

# vim: expandtab sw=4 ts=4
