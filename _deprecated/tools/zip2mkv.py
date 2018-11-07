

import shutil
import sys
import os.path
from .logparser2 import sensor_gen
from zipfile import ZipFile
import tempfile
import subprocess


def name2mtime(metapath, copy_func):
    srcdir = os.path.dirname(metapath)
    destdir = srcdir+'/img'
    os.mkdir(destdir)
    start = None
    for ts, id, (filename, _) in sensor_gen(metapath, [b"camera"]):
        if start is None:
            start = ts
        filename = os.path.basename(filename)
        copy_func(srcdir+'/'+filename, destdir+'/'+filename)
        os.utime(destdir+'/'+filename, (ts, ts))
        print("{:.3f}".format((ts-start)*1000))
    return destdir


def jpegtrans(srcpath, destpath):
    subprocess.call(args=['jpegtran','-optimize','-outfile',destpath,srcpath])


def ffmpeg(srcdir, destpath, attachments=[]):
    a = []
    for i in attachments:
        a.extend(('-attach', i))
    subprocess.call(['ffmpeg', '-y', '-hide_banner', '-nostats',
                     '-ts_from_file', '2', '-pattern_type', 'glob', '-i', srcdir+'/*.jpg',
                     '-metadata:s:t', 'mimetype=text/plain'] + a + ['-c:v', 'copy', destpath ])
    return destpath


def main(filepath):
    try:
        dirpath = tempfile.mkdtemp()
        with ZipFile(filepath, 'r') as myzip:
            myzip.extractall(dirpath)

        metapath = [p for p in os.listdir(dirpath) if p.startswith("meta_")][0]
        destdir = name2mtime(dirpath+'/'+metapath, jpegtrans)
        destfile = os.path.splitext(os.path.basename(filepath))[0]+'.mkv'
        attachments = []
        print('#'*80)
        for p in os.listdir(dirpath):
            if not p.endswith('.jpg') and not os.path.isdir(dirpath+'/'+p):
                attachments.append(dirpath+'/'+p)
        mkv = ffmpeg(destdir, destfile, attachments)
        return mkv
    finally:
        shutil.rmtree(dirpath)


if __name__ == "__main__":
    output = main(sys.argv[1])
    print("output: {}".format(output))
