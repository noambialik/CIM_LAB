from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler

import BenchTurnFunctions


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

# Server to connect clients to the API
# Based on Scorpy's SimpleXMLRPCServer (server.py)
class Server:
    quit = False

    def __init__(self):
        self.server = SimpleXMLRPCServer(("0.0.0.0", 8002),
                                         requestHandler=RequestHandler, logRequests=False)
        self.server.register_introspection_functions()
        self.server.register_function(self.shutdown)
        self.server.register_instance(BenchTurnFunctions.BenchTurnFunctions())
        self.server.register_function(self.test)
        print("started main server")
        while not self.quit:
            self.server.handle_request()
        self.server.stop_all()

    def test(self):
        print("test")
        return True

    def shutdown(self):
        print "exiting ..."
        self.quit = True
        return 0


Server()
