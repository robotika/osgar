"""
  Simple application demonstrating AsyncNode receiving and replying to messages.
"""
from osgar.node import AsyncNode


class EchoApp(AsyncNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw')

    async def on_raw(self, data):
        text = data.decode('utf-8', errors='replace').strip()
        response = f"ECHO: {text}\n".encode('utf-8')
        self.publish('raw', response)

# vim: expandtab sw=4 ts=4
