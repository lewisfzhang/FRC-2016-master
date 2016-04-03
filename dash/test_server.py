import sys
import time
import logging

sys.path.append("pynetworktables-2015.3.2-py2.7.egg")
from networktables import NetworkTable

logging.basicConfig(level=logging.DEBUG)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")

def valueChanged(table, key, value, isNew):
    print("Value Changed", table.path, key, value)
table.addTableListener(valueChanged)

i = 0
while True:
    table.putNumber("server_count", i)
    i += 1
    time.sleep(1)
