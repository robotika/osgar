"""
  Node - processing unit
"""
import asyncio
from threading import Thread

from osgar.bus import BusShutdownException


class Node(Thread):
    """
       Parent class for all processing "nodes".
    """
    def __init__(self, config, bus):
        super().__init__(daemon=True)
        self.bus = bus
        self.time = None
        self.verbose = False  # set externally via "osgar.replay --verbose ..."

    def publish(self, channel, data):
        return self.bus.publish(channel, data)

    def listen(self):
        return self.bus.listen()

    def sleep(self, secs):
        self.bus.sleep(secs)

    def is_bus_alive(self):
        return self.bus.is_alive()

    def update(self):
        timestamp, channel, data = self.bus.listen()
        self.time = timestamp
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(data)
        else:
            assert False, channel  # unknown
        return channel

    def run(self):
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class AsyncNode(Node):
    """
       Parent class for processing nodes executing an asyncio event loop.
    """
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.loop = None
        self._main_task = None

    async def listen_async(self):
        """Asynchronously listen for the next message from the bus."""
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self.bus.listen)

    async def update_async(self):
        """Asynchronously receive one message from the bus and dispatch to handler."""
        timestamp, channel, data = await self.listen_async()
        self.time = timestamp
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            if asyncio.iscoroutinefunction(handler):
                await handler(data)
            else:
                handler(data)
        else:
            assert False, channel  # unknown
        return channel

    def update(self):
        """Synchronous update, allowing AsyncNode to be run in synchronous replay/tests."""
        timestamp, channel, data = self.bus.listen()
        self.time = timestamp
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            if asyncio.iscoroutinefunction(handler):
                asyncio.run(handler(data))
            else:
                handler(data)
        else:
            assert False, channel  # unknown
        return channel

    async def sleep_async(self, secs):
        """Asynchronous non-blocking sleep."""
        await asyncio.sleep(secs)

    async def listen_loop(self):
        """Continuous async loop listening to the bus and dispatching messages."""
        try:
            while self.is_bus_alive():
                await self.update_async()
        except BusShutdownException:
            pass

    async def main(self):
        """Main entry point for async tasks. Default implementation runs listen_loop."""
        await self.listen_loop()

    def run(self):
        """Thread target that initializes and runs the asyncio event loop."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self._main_task = self.loop.create_task(self.main())
            self.loop.run_until_complete(self._main_task)
        except (asyncio.CancelledError, BusShutdownException):
            pass
        finally:
            try:
                pending = [t for t in asyncio.all_tasks(self.loop) if not t.done()]
                for task in pending:
                    task.cancel()
                if pending:
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                if hasattr(self.loop, "shutdown_default_executor"):
                    self.loop.run_until_complete(self.loop.shutdown_default_executor())
            finally:
                self.loop.close()
                self.loop = None

    def request_stop(self):
        """Request stop of the node and graceful shutdown of event loop."""
        self.bus.shutdown()
        if self.loop is not None and self.loop.is_running():
            self.loop.call_soon_threadsafe(self._cancel_main_task)

    def _cancel_main_task(self):
        if self._main_task is not None and not self._main_task.done():
            self._main_task.cancel()

# vim: expandtab sw=4 ts=4
