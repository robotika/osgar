# vim: set fileencoding=utf-8 et sts=4 sw=4:


import matplotlib.pyplot as plt
from logparser import MsgInterpreter
from sys import exit as sysexit

def filter(data, ratio):
    val=data[0]
    for d in data:
        val= ratio *d+(1-ratio) *val
        yield val
class filtration:
    def __init__(self,log_fn, ratio=0.5):
        self.ratio=ratio
        self.step=0.1
        vars=[]
        lastinfo=None
        for time, msg_origin, msg in MsgInterpreter(logfn):
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
                    vars.append((time,rspeed))
        self.x,self.y=list(zip(*vars[900:1000]))

        self.fig=plt.figure()
        self.fig.canvas.mpl_connect('key_release_event', self.keyfn)
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.set_xlabel('time')
        self.ax.grid()
        self.ax.set_ylabel('var', color='b')
        self.draw()
        plt.show(block=True)


    def draw(self):
        self.ax.clear()
        self.ax.plot(self.x, self.y, color='b',marker='.')
        self.ax.plot(self.x, list(filter(self.y, self.ratio)), color='r')
        plt.title("%.3f, step %f"%(self.ratio,self.step))
        plt.draw()

    def keyfn(self, event):
        if event.key == 'up':
            self.ratio+=self.step
        elif event.key == 'right':
            self.step*=0.1
        elif event.key == 'left':
            self.step*=10
        elif event.key == 'down':
            self.ratio-=self.step
        elif event.key == 'escape':
            sysexit()
        self.draw()





if __name__=="__main__":
    from sys import argv
    logfn=r'..\logs\can_160823_183916.log'
    if len(argv)>1:
        logfn=argv[1]
    filtration(logfn)

