"""
  AsyncNode - processing unit executing an asyncio event loop
"""
import asyncio
import threading
from osgar.node import Node
from osgar.bus import BusShutdownException


class AsyncNode(Node):
    """
       Parent class for processing nodes executing an asyncio event loop.
    """
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.loop = None
        self._main_task = None
        self._force_stop_requested = threading.Event()
        self._cancel_sent = False

    def publish(self, channel, data):
        return self.bus.publish(channel, data)

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
            while True:
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
            if self._force_stop_requested.is_set():
                self._cancel_main_task()
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
        """Drain queued input, then let main() finish its async cleanup."""
        self.bus.shutdown()

    def force_stop(self):
        """Cancel blocked async work after the graceful shutdown deadline."""
        self.bus.shutdown()
        self._force_stop_requested.set()
        loop = self.loop
        if loop is not None:
            try:
                loop.call_soon_threadsafe(self._cancel_main_task)
            except RuntimeError:
                if not loop.is_closed():
                    raise

    def _cancel_main_task(self):
        if self._main_task is not None and not self._main_task.done() and not self._cancel_sent:
            self._cancel_sent = True
            self._main_task.cancel()

# vim: expandtab sw=4 ts=4
