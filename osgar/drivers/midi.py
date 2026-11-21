"""Capture Linux ALSA raw MIDI and play it back from an OSGAR log."""

import argparse
import os
from select import select
from time import monotonic, sleep

from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id
from osgar.node import Node


def raw_midi_device(card):
    """Resolve an ALSA card ID to its first raw MIDI device."""
    path = os.path.join('/proc/asound', card)
    if not os.path.islink(path):
        raise FileNotFoundError(f'ALSA card {card!r} not found at {path}')
    card_number = os.readlink(path)[len('card'):]
    return f'/dev/snd/midiC{card_number}D0'


class Midi(Node):
    """Publish incoming MIDI byte chunks on midi_data."""

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.bus.register('midi_data')
        self.card = config['card']
        self.fd = None

    def start(self):
        self.fd = os.open(raw_midi_device(self.card), os.O_RDONLY | os.O_NONBLOCK)
        try:
            super().start()
        except BaseException:
            os.close(self.fd)
            raise

    def run(self):
        try:
            while self.is_bus_alive():
                readable, _, _ = select([self.fd], [], [], 0.1)
                if not readable:
                    continue
                try:
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    break
                self.publish('midi_data', data)
        finally:
            os.close(self.fd)


def play(filename, device, stream='midi.midi_data'):
    stream_id = lookup_stream_id(filename, stream)
    with LogReader(filename, only_stream_id=stream_id) as log:
        first_timestamp = None
        with os.fdopen(os.open(device, os.O_WRONLY), 'wb', buffering=0) as output:
            for timestamp, _, raw in log:
                data = memoryview(deserialize(raw))
                if first_timestamp is None:
                    first_timestamp = timestamp
                    start = monotonic()
                deadline = start + (timestamp - first_timestamp).total_seconds()
                sleep(max(0, deadline - monotonic()))
                while data:
                    data = data[output.write(data):]


def main():
    parser = argparse.ArgumentParser(description='Play recorded MIDI through a Linux ALSA raw MIDI device')
    parser.add_argument('log', help='OSGAR log file')
    parser.add_argument('--card', default='UX16', help='ALSA card ID from /proc/asound/cards (default: UX16)')
    parser.add_argument('--stream', default='midi.midi_data', help='OSGAR MIDI stream name or ID')
    args = parser.parse_args()
    try:
        play(args.log, raw_midi_device(args.card), args.stream)
    except (OSError, ValueError) as error:
        parser.exit(1, f'{error}\n')
    except KeyboardInterrupt:
        parser.exit(130)


if __name__ == '__main__':
    main()
