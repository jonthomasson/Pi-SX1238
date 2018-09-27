#test_reg.py

import sx1238
from sx1238_registers import *
import datetime
import time

test = sx1238.SX1238(20)
print "class initialized"

print "setting opmode to standby"
test.write_register(0x01, 1)

print "opmode is..."
print test.read_register(0x01)

print "reading all registers"
results = test.read_all_regs()
for result in results:
    print result

print "Checking temperature"
print test.read_temperature(0)

print "recieving"
while True:
    test.receive_begin()
    while not test.receive_done():
        time.sleep(.1)
    print "%s from %s RSSI:%s" % ("".join([chr(letter) for letter in test.data]), test.sender_address, test.rssi)
 

print "shutting down"
test.shutdown()