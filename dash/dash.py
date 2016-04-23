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
clientInitMessages = {}
class BridgeServer(WebSocket):
    def handleConnected(self):
        print("Connected", self.address)
        activeBridges.add(self)
        for _, (tableName, key, value) in clientInitMessages.iteritems():
            self.sendValue(tableName, key, value)

    def handleMessage(self):
        print("Got Message:", self.data)
        try:
            jsonPayload = json.loads(self.data)
            if jsonPayload["table"] != table.path:
                print("Unknown table")
                return
            if jsonPayload["type"] == "string":
                table.putString(jsonPayload["key"], jsonPayload["value"])
            elif jsonPayload["type"] == "bool":
                table.putBoolean(jsonPayload["key"], jsonPayload["value"])
        except Exception as e:
            print e

    def handleClose(self):
        print("Closed", self.address)
        activeBridges.remove(self)

    def sendValue(self, tableName, key, value):
        jsonPayload = {}
        jsonPayload["table"] = tableName
        jsonPayload["key"] = key
        jsonPayload["value"] = value
        jsonString = u"" + json.dumps(jsonPayload)
        self.sendMessage(jsonString)

def valueChanged(table, key, value, isNew):
    clientInitMessages[(table, key)] = (table.path, key, value)
    for bridge in activeBridges:
        bridge.sendValue(table.path, key, value)


table.addTableListener(valueChanged)

server = SimpleWebSocketServer("", 8000, BridgeServer)

def close_sig_handler(signal, frame):
    server.close()
    sys.exit()

signal.signal(signal.SIGINT, close_sig_handler)
server.serveforever()
