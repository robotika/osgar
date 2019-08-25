from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

CELL_PREFIX = "PhoneArtifact"

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Parse logged WiFi AP scans and extract "Cell Phone" artifact')
    parser.add_argument('logfile', help='recorded log file')
    args = parser.parse_args()

    only_stream = lookup_stream_id(args.logfile, 'wifi.wifiscan')
    best = None
    with LogReader(args.logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            wifi_list = deserialize(data)
            for name, signal in wifi_list:
                if name.startswith(CELL_PREFIX):
                    print(timestamp, name, signal)
                    if best is None or signal > best[2]:
                        best = timestamp, name, signal
    print('--------------------')
    if best is not None:
        print('best:', best[0], best[1], best[2])
    else:
        print("No Cell Phone, sorry")

# vim: expandtab sw=4 ts=4

