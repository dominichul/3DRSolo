#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import time
import math
import sys

# Connect to UDP endpoint (and wait for default attributes to accumulate)
#target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
target = sys.argv[1] if len(sys.argv) >= 2 else '127.0.0.1:14549'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)


#Allows vehicle home location to be read
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()


#Arm vehicle and takeoff/hover at target altitude
def arm_and_takeoff(aTargetAltitude):

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)

    # Wait until the vehicle reaches a safe height before processing the goto (next command should not be executed immediately.
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Break once just below target altitude
            print "Reached target altitude"
            break
        time.sleep(1)


#Arm and take of to altitude of 13 meters
arm_and_takeoff(13)


"""
Using the MAV_CMD_CONDITION_YAW MAVLINK message, condition_yaw allows the user to set a heading direction for the drone. 
This can be set with an absolute heading value, or a heading value relative to the current heading. By default, unless otherwise specified, the heading is set as an absolute value (0-360, 0 is North)
"""

def condition_yaw(heading, relative=False):

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


"""
Send MAV_CMD_DO_SET_ROI message to set the heading towards a target location
"""

def set_roi(location):

    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns an object of the same type as original_location, moving 'dNorth' meters North, and 'dEast' meters East from the original locastion
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    Formula using the haversine formula as seen at: http://www.movable-type.co.uk/scripts/latlong.html
    """
    #dlat = aLocation2.lat - aLocation1.lat
    #dlong = aLocation2.lon - aLocation1.lon
    #return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    R = 6371000.0
    dLat = math.radians(aLocation2.lat - aLocation1.lat)
    dLon = math.radians(aLocation2.lon - aLocation1.lon)
    lat1 = math.radians(aLocation1.lat)
    lat2 = math.radians(aLocation2.lat)

    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    d = R * c

    return d;


"""
Returns the bearing between the two LocationGlobal objects passed as parameters.
Not being used currently
"""	

def get_bearing(aLocation1, aLocation2):

    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;



"""
Using the SET_POSITION_TARGET_GLOBAL_INT MAVLINK command, a request is made to fly to a specified location using object type LocationGlobal
"""


def goto_position_target_global_int(aLocation):

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto_position_target_local_ned(north, east, down):
    """	
    Another goto MAVLINK command. Allows for further specification of desired altitude relative to the home position. (It is important to use negative values for the 'down' parameter. A value of 10 would suggest 10 meters below the home position. -10 would suggest 10 meters above)

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto(tLocation,dNorth, dEast, gotoFunction=vehicle.simple_goto):
"""
The method first calculates a target location given currentLocation and dNorth/dEast values.

tLocation represents the desired heading of the drone and is set through set_roi method. It is also used to point the camera to the desired location through the gimbal and vehicle.gimbal.target_location method.

The default goto method used is dronekit's .goto method. A separate method may be used.

As the vehicle approaches the target location, remaining distance is calculated every 2 seconds, as well as updating the gimbal's target location. Once the remaining distance is within 1 meter, the script sleeps 2 seconds before executing the next command/function
		
"""
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    set_roi(tLocation)

    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        
        #dist_tLocation = get_distance_metres(vehicle.location.global_relative_frame, tLocation)

        #gimbalAngle = math.degrees(math.atan(dist_tLocation/vehicle.location.global_relative_frame.alt))

        #gimbalAngle = -90 + gimbalAngle
        #vehicle.gimbal.rotate(gimbalAngle,0,0)

        vehicle.gimbal.target_location(tLocation)

        print "Distance to target: ", remainingDistance
        if remainingDistance < 1: #Just below target, in case of undershoot.
            time.sleep(2)
            print "Reached target"
            break;
        time.sleep(2)




def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    *Not currently used
   
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
    


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

	Same use as send_ned_velocity.

	*Not currently used
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    


home = vehicle.home_location
print "Home location is: %s" % home

# sleep so we can see the change in map
time.sleep(10)

vehicle.groundspeed=3


targetLocation = get_location_metres(home, 0, -50)

print("Target Position: 0m North/South, 50m West")
goto(targetLocation , 0, -50)

#cLocation = vehicle.location.global_relative_frame
#cLocation.alt += 10

#print "Increasing Altitude to: %s m" % cLocation.alt 
#vehicle.simple_goto(cLocation)

time.sleep(10)

print("Completing square route around target")
print("10m West, 10m North")
goto(targetLocation ,15,-15)

print("20m East, 0m North/South")
goto(targetLocation, 0,30)

print("0m West/East, 20m South")
goto(targetLocation, -30,0)

print("20m West, 0m North/South")
goto(targetLocation, 0,-30)

print("0m West/East, 20m North")
goto(targetLocation, 30,0)

print "Returning to Launch"

set_roi(home)
vehicle.gimbal.target_location(home)
vehicle.mode = VehicleMode("RTL")

time.sleep(5)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

