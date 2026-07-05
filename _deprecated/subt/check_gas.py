from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Parse logged detected_gas and extract "Gas" artifact')
    parser.add_argument('logfile', help='recorded log file')
    args = parser.parse_args()

    only_stream = lookup_stream_id(args.logfile, 'gas_detector.co2')
    best = None
    with LogReader(args.logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            co2 = deserialize(data)
            if best is None or co2 > best[1]:
                print(timestamp, co2)
                best = timestamp, co2
    print('--------------------')
    if best is not None:
        print('best:', best[0], best[1])
    else:
        print("No Gas, sorry")

# vim: expandtab sw=4 ts=4

