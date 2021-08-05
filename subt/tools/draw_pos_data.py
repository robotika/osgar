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
    for name, arr in robots.items():
        x = [xyz[0] for t, xyz in arr]
        y = [xyz[1] for t, xyz in arr]
        plt.plot(x, y, '-', label=name)
    plt.axes().set_aspect('equal', 'datalim')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', help='folder with *-pos.data files')
    args = parser.parse_args()
    robots = read_all(args.folder
                      )
    draw(robots)
