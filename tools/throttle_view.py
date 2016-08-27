# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals

import matplotlib.pyplot as plt
from logparser import MsgInterpreter

def analyze_throttle(log_fn):
    throttle=[]
    control=[(0,0)]
    lastcontrol=0
    speeds=[]
    avgspeeds=[]
    lastinfo=None
    avgspeed=0
    for time, msg_origin, msg in MsgInterpreter(logfn):
        if msg.name=='throtleinfo':
            throttle.append((time,msg.position))
        if msg.name=='motorcontrol':
            cn={3:-1, 0:0, 0xc:1}[msg.direction]
            control.append((time,lastcontrol))
            lastcontrol=cn
            control.append((time,cn))
        if msg.name == 'encoderinfo':
            if not lastinfo:
                lastinfo=msg
            diffL=msg.left-lastinfo.left
            diffR=msg.right-lastinfo.right
            if abs(diffL)>10 or abs(diffR)>10:
                #print(diffL, diffR, lastinfo.left, lastinfo.right, msg.left,msg.right)
                pass
            else:
                lastinfo=msg
                rspeed=(diffR+diffL)/2.
                speeds.append((time,rspeed))
                avgspeed=avgspeed*0.8+rspeed*0.2
                avgspeeds.append((time, avgspeed))

    fig=plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    fig.subplots_adjust(right=0.75)
    ax.set_xlabel('time')
    ax.grid()
    ax2 = ax.twinx()
    ax2.set_ylabel('pos', color='b')
    ax.set_ylabel('control', color='r')
    ax.set_ylim(-1.1,1.1)
    ax2.plot(*zip(*throttle), label='pos', color='b',marker='.')



    SCALE_NEW=0.2
    th_filter=throttle[0][1]
    fthrottle=[]
    #time,fthrottle=zip(*throttle)

    for time, th in throttle:
            th_filter =  SCALE_NEW*th + (1.0 - SCALE_NEW)*th_filter
            fthrottle.append((time,th_filter))
    ax2.plot(*zip(*fthrottle), label='pos', color='m', marker='.')


    ax2.axhline(y=14500, alpha=0.5)
    ax2.axhline(y=16500, alpha=0.5)
    ax2.axhline(y=19000, alpha=0.5)
    ax.plot(*zip(*control), label='control', color='r')

    ax3 = ax.twinx()
    ax3.spines["right"].set_position(("axes", 1.2))
    ax3.set_ylabel('speed', color='g')
    ax3.plot(*zip(*avgspeeds), label='speed', color='g')

    plt.show(block=True)


if __name__=="__main__":
    from sys import argv
    logfn=r'..\logs\can_160823_183916.log'
    if len(argv)>1:
        logfn=arv[1]
    analyze_throttle(logfn)

