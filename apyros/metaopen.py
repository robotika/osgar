"""
   only wrapper for ZIP files
"""

import os
from zipfile import ZipFile


def metaopen(filename, mode='r'):
    """Wrapper for opening files optionally from ZIP"""
    assert mode in ['r', 'rb'], mode  # just to know what modes we use
    if '.zip' in filename:
        return ZipFile(os.path.dirname(filename)).open(os.path.basename(filename))
    else:
        return open(filename, mode)
# vim: expandtab sw=4 ts=4 

