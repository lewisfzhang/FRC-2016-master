import signal
import sys
import time
import json

sys.path.append("pynetworktables-2015.3.2-py2.7.egg")
from networktables import NetworkTable

sys.path.append("SimpleWebSocketServer-0.1.0-py2.7.egg")
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

if len(sys.argv) != 2:
    print("Error: specify a Network Table IP to connect to!")
    exit(-1)

ip = sys.argv[1]
print("Connecting to ip: %s" % ip)

NetworkTable.setClientMode()
NetworkTable.setIPAddress(ip)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")

class TableConnectionListener:
    def connected(self, table):
        print("Connected to", table.getRemoteAddress(), table)

    def disconnected(self, table):
        print("Disconnected", table)

tableConnectionListener = TableConnectionListener()
table.addConnectionListener(tableConnectionListener)

activeBridges = set()
class BridgeServer(WebSocket):
    def handleConnected(self):
        print("Connected", self.address)
        activeBridges.add(self)

    def handleMessage(self):
        print("Got Message:", self.data)

    def handleClose(self):
        print("Closed", self.address)
        activeBridges.remove(self)

def valueChanged(table, key, value, isNew):
    jsonPayload = {}
    jsonPayload["table"] = table.path
    jsonPayload["key"] = key
    jsonPayload["value"] = value
    jsonString = u"" + json.dumps(jsonPayload)
    for bridge in activeBridges:
        bridge.sendMessage(jsonString)

table.addTableListener(valueChanged)

server = SimpleWebSocketServer("", 8000, BridgeServer)

def close_sig_handler(signal, frame):
    server.close()
    sys.exit()

signal.signal(signal.SIGINT, close_sig_handler)
server.serveforever()
