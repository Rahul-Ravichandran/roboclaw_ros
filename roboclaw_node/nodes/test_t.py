#!/usr/bin/env python

import time as time
import roboclaw_driver.roboclaw_driver as roboclaw

#Linux comport name
roboclaw.Open("/dev/ttyACM2",38400)

address = 0x80

version = roboclaw.ReadVersion(address)
if version[0]==False:
	print "GETVERSION Failed"
else:
	print repr(version[1])

print roboclaw.ReadMinMaxMainVoltages(address)
print roboclaw.ReadMinMaxLogicVoltages(address)
print roboclaw.ReadM1MaxCurrent(address)
print roboclaw.ReadM2MaxCurrent(address)
print roboclaw.ReadTemp(address)
print roboclaw.ReadTemp2(address)
print roboclaw.ReadSpeedM1(address)
print roboclaw.ReadSpeedM2(address)

# roboclaw.ForwardM1(address, 30)
# time.sleep(1)
# print("fwd done")
# roboclaw.ForwardM1(address, 63)
# time.sleep(1)
# roboclaw.ForwardM1(address, 93)
# time.sleep(1)
# roboclaw.ForwardM1(address, 0)
# print("end")