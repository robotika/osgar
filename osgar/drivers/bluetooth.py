"""
  BlueTooth scanner of nearby Device ID, names and signal strength
"""
# source: https://www.perplexity.ai/search/i-would-like-to-write-a-small-vWtazaiBTHi9qf8FptD0iw
#  uv add bleak

import asyncio
from bleak import BleakScanner
from osgar.node import Node


async def run():
    print('Scanning ...')
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)
    return devices


class BluetoothScanner(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('bt_scan')

    def on_trigger(self, data):
        devices = asyncio.run(run())
        serialized = [
            {
                "address": d.address,
                "name": d.name,
                "rssi": d.rssi
            }
            for d in devices]
        self.publish('bt_scan', serialized)
