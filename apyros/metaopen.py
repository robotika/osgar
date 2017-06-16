"""
   only wrapper for ZIP files
"""

import os
import zipfile
import StringIO


def metaopen(filename, mode='r'):
    """Wrapper for opening files optionally from ZIP"""
    assert mode in ['r', 'rb'], mode  # just to know what modes we use
    if '.zip' in filename:
        data = zipfile.ZipFile(os.path.dirname(filename)).read(os.path.basename(filename))
        print len(data)
        return StringIO.StringIO(data)
    else:
        return open(filename, mode)
# vim: expandtab sw=4 ts=4 

