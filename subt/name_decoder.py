"""
  Virtual SubT Challenge - decoder of robot name

The name is based on combination of capital letters and numbers.
New coding:
  <name letter (any)> ([<duration>] <action type>)+
  where supported actions are (first letter) Left, Right, Center, Wait, Home

The last action is by default Home. There is no default direction.
The time for Home is optional (default is 2 times sum of previous exploration).
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
        'W': 'wait',
        'H': 'home'
    }
    parts = split_multi(robot_name[1:], options.keys())
    ret = []
    sum_t = 0
    entered = False
    for part in parts:
        action = options[part[-1]]
        if len(part) > 1:
            t = int(part[:-1])
        else:
            assert action == 'home', action  # missing time
            t = 2 * sum_t
        if action in ['left', 'right', 'center']:
            # exploration commands
            if not entered:
                ret.append(('enter', None))
                entered = True
            sum_t += t
        ret.append((action, t))
        if action == 'home':
            entered = False
            sum_t = 0
    ret.append(('home', sum_t * 2))
    return ret

# vim: expandtab sw=4 ts=4
