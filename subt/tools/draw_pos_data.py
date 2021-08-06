"""
  Draw *-pos.data files stored in simulation log
"""

from pathlib import Path

import cv2
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


def draw(robots, create_video_path=None):
    fig, ax = plt.subplots()
    size = 0
    for name, arr in robots.items():
        x = [xyz[0] for t, xyz in arr]
        y = [xyz[1] for t, xyz in arr]
        plt.plot(x, y, '-', label=name)
        size = max(size, len(x))
    ax.set_aspect(1.0)
    plt.legend()

    # The function to be called anytime a slider's value changes
    def update(val):
        ax.clear()
        for name, arr in robots.items():
            x = [xyz[0] for t, xyz in arr]
            y = [xyz[1] for t, xyz in arr]
            index = min(int(val), len(x) - 1)
            ax.plot(x[:index+1], y[:index+1], '-', label=name)
            ax.scatter([x[index]], [y[index]], s=50)
        fig.canvas.draw_idle()

    if create_video_path is not None:
        assert create_video_path.endswith(".mp4"), create_video_path

        tmp_image = "tmp_img.png"
        writer = None
        fps = 10
        for i in range(size):
            update(i)
            plt.savefig(tmp_image)
            img = cv2.imread(tmp_image, 1)
            if writer is None:
                height, width = img.shape[:2]
                writer = cv2.VideoWriter(create_video_path,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps,
                                         (width, height))
            writer.write(img)
        if writer is not None:
            writer.release()
        return  # i.e. no interactive session

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


    slider.on_changed(update)

    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', help='folder with *-pos.data files')
    parser.add_argument('--create-video', help='output path (*.mp4) for generated video')
    args = parser.parse_args()
    robots = read_all(args.folder)
    draw(robots, create_video_path=args.create_video)
