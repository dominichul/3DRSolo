from dronekit import connect, VehicleMode, time
import sys
import time

# Connect to UDP endpoint (and wait for default attributes to accumulate)
target = sys.argv[1] if len(sys.argv) >= 2 else '127.0.0.1:14555'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

#vehicle.parameters['ARMING_CHECK']=-9

print "Waiting for arming..."

while not vehicle.armed:
	time.sleep(4)


while vehicle.armed:

# Get all vehicle attributes (state)
	print "Vehicle state:"
	print " Relative Altitude: %" vehicle.location.global_relative_frame.alt
	print " Velocity: %s" % vehicle.velocity
	print " Battery Percent: %s" % vehicle.battery.level
	print " Groundspeed: %s" % vehicle.groundspeed
	print " Airspeed: %s" % vehicle.airspeed
	print " Mode: %s" % vehicle.mode.name
	time.sleep(2)

sys.stdout.flush()


vehicle.close()
print "Done."

