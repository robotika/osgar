"""
  Draw *-pos.data files stored in simulation log
"""

from pathlib import Path

from matplotlib import pyplot as plt


def read_pos_data(filename):
    """
    Read data from file which looks like (sec, nsec, x, y, z)
        0 8000000 -14 0 0.149843
        1 12000000 -13.9997 -1.6e-05 0.117777
        2 16000000 -13.9997 -1.9e-05 0.117841
    """
    arr = []
    for line in open(filename):
        sec, nsec, x, y, z = [float(a) for a in line.split()]
        arr.append((sec + nsec/1_000_000_000, (x, y, z)))
    return arr


def read_all(folder):
    ret = {}
    for path in Path(folder).glob('*-pos.data'):
        name = path.stem[:-4]  # cutoff "-pos"
        print(name)
        ret[name] = read_pos_data(path)
    return ret


def draw(robots):
    fig, ax = plt.subplots()
    pose = {}
    size = 0
    for name, arr in robots.items():
        x = [xyz[0] for t, xyz in arr]
        y = [xyz[1] for t, xyz in arr]
        plt.plot(x, y, '-', label=name)
        index = len(x) - 1
        pose[name] = plt.scatter([x[index]], [y[index]], s=50)
        size = max(size, len(x))
#    plt.axes().set_aspect('equal', 'datalim')
#    plt.legend()

    axcolor = 'lightgoldenrodyellow'
    # Make a horizontal slider to control the frequency.
    axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
    slider = plt.Slider(
        ax=axfreq,
        label='time [s]',
        valmin=0.0,
        valmax=size,
        valinit=0.0,
    )

    # The function to be called anytime a slider's value changes
    def update(val):
        ax.clear()
#        line.set_ydata(f(t, amp_slider.val, freq_slider.val))
        for name, arr in robots.items():
            x = [xyz[0] for t, xyz in arr]
            y = [xyz[1] for t, xyz in arr]
            ax.plot(x, y, '-', label=name)
            index = min(int(val), len(x) - 1)
           # pose[name].set_offset([[x[index]], [y[index]]])
#            pose[name] = plt.scatter([x[index]], [y[index]], s=50)
            ax.scatter([x[index]], [y[index]], s=50)
            pose[name].set_xdata = [x[index]]
            pose[name].set_ydata = [y[index]]
        fig.canvas.draw_idle()
#        fig.canvas.draw()
#        fig.canvas.flush_events()

#    update(1000)

    slider.on_changed(update)

    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', help='folder with *-pos.data files')
    args = parser.parse_args()
    robots = read_all(args.folder)
    draw(robots)
