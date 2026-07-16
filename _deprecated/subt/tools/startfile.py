import os
import os.path
import subprocess
import sys

if sys.platform[:3] == "win":

    def main(filepath):
        normpath = os.path.normpath(filepath)
        os.startfile(normpath)

else:

    def main(filepath):
        subprocess.Popen(['xdg-open', filepath], close_fds=True, start_new_session=True)


if __name__ == "__main__":
    main(sys.argv[1])
