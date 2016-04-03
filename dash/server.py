import sys
import time
import logging

sys.path.append("pynetworktables-2015.3.2-py2.7.egg")
from networktables import NetworkTable

logging.basicConfig(level=logging.DEBUG)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")

i = 0
while True:
    table.putNumber("server_count", i)
    print("looping %s" % i)
    i += 1
    time.sleep(1)
