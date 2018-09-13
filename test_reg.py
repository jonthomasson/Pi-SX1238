#test_reg.py

import sx1238

sx1238.init()

regval = sx1238.read_register(0x01) #read regopmode

print("reg opmode = ", regval)

regval = sx1238.read_register(0x33) #read regnodeadrs

print("reg node address = ", regval)

print("writing new node address = 10")
sx1238.write_register(0x33, 10) #write register regnodeadrs

regval = sx1238.read_register(0x33) #read regnodeadrs

print("reg node address = ", regval)