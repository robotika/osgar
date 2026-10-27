"""
  AsyncEcho - example driver using AsyncNode providing an asynchronous TCP server.
"""
import asyncio
import threading
from osgar.asyncnode import AsyncNode


class AsyncEcho(AsyncNode):
    """
    An async TCP server driver that:
    - Listens for TCP client connections on host:port.
    - Publishes incoming data from clients to the OSGAR bus on 'raw'.
    - Receives data from the OSGAR bus on 'raw' and broadcasts to connected TCP clients.
    """
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw')
        self.host = config.get('host', '127.0.0.1')
        self.port = config.get('port', 8888)
        self.clients = set()
        self.server = None
        self.ready = threading.Event()

    async def main(self):
        try:
            self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
            sockets = self.server.sockets
            if sockets:
                self.port = sockets[0].getsockname()[1]
            self.ready.set()
            async with self.server:
                # start_server already accepts clients. The bus loop owns the
                # lifetime so graceful close can finish without cancelling main.
                await self.listen_loop()
        except asyncio.CancelledError:
            pass
        except Exception:
            self.ready.set()
            raise
        finally:
            for writer in list(self.clients):
                try:
                    writer.close()
                    await writer.wait_closed()
                except Exception:
                    pass
            self.clients.clear()

    async def handle_client(self, reader, writer):
        self.clients.add(writer)
        try:
            while not reader.at_eof():
                data = await reader.read(1024)
                if not data:
                    break
                self.publish('raw', data)
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        finally:
            self.clients.discard(writer)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def on_raw(self, data):
        """Broadcast data received from the OSGAR bus to all connected clients."""
        if not isinstance(data, (bytes, bytearray)):
            if isinstance(data, str):
                data = data.encode('utf-8')
            else:
                data = str(data).encode('utf-8')

        for writer in list(self.clients):
            try:
                writer.write(data)
                await writer.drain()
            except Exception:
                self.clients.discard(writer)

    def request_stop(self):
        super().request_stop()
        self.ready.set()

# vim: expandtab sw=4 ts=4
