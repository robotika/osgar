from xml.sax.handler import ContentHandler
from xml.sax import make_parser, saxutils

import base64
import zlib


class GazeboHandler(ContentHandler):
    def __init__(self):
        super().__init__()
        self.active = False
        self.index = 0

    def startElement(self, name, attrs):
        assert name in ['gazebo_log', 'header', 'log_version', 'gazebo_version',
                'rand_seed', 'chunk', 'log_start', 'log_end'], name

        if name == 'chunk':
            assert attrs['encoding'] == 'zlib', attrs['encoding']
            self.data = ''
        self.active = (name == 'chunk')

    def characters(self, data):
        if self.active:
            self.data += data

    def endElement(self, name):
        if name == 'chunk':
            subtree = zlib.decompress(base64.b64decode(self.data))
            if self.index == 100:
                print(subtree.decode('ascii'))
            self.index += 1


if __name__ == "__main__":
    import sys
    handler = GazeboHandler()

    saxparser = make_parser()
    saxparser.setContentHandler(handler)

    with open(sys.argv[1]) as f:
        saxparser.parse(f)


# vim: expandtab sw=4 ts=4
