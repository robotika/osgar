"""
  Wrapper & XML RPC (both client and server)
"""
import xmlrpc.client


class LogTransportParser:
    def __init__(self, bus, parser):
        self.bus = bus
        self.p = parser

    def feed(self, data):
        self.bus.publish('raw', data)
        self.p.feed(data)

    def close(self):
        self.p.close()


class LogTransport(xmlrpc.client.Transport):
    def __init__(self, bus):
        self.bus = bus
        super().__init__()

    def make_connection(self, host):
        self.bus.publish('make_connection', host)
        h = super().make_connection(host)
        return h

    def send_request(self, connection, handler, request_body, debug):
        self.bus.publish('send_request', request_body)
        return super().send_request(connection, handler, request_body, debug)

    def getparser(self):
        p, u = super().getparser()
        return LogTransportParser(self.bus, p), u


class LogServerProxy(xmlrpc.client.ServerProxy):
    def __init__(self, bus, uri):
        super().__init__(uri, LogTransport(bus))


# vim: expandtab sw=4 ts=4

