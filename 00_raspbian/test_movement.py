#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# setting up common modules used in the program
from __future__ import print_function
import socket
import time
import sys
import os

# setting up dronekit and pymavlink modules used in the program
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# check os compatibility
if os.name == 'nt':

    print ("\nThis codes only compatible with Linux OS!")
    exit()

# clear the screen
os.system("clear")

# check python environment compatibility
if sys.version_info[0] > 2:

    print ("\nThis codes only compatible with Python 2 environment!")
    exit()

# suppressing messages or errors from dronekit
def suppress_dronekit_messages(x):

    pass

# define take off sequence
def arm_and_takeoff(height):
    
    a = 0
    b = 0.95

    # check if drone is armable or not
    while not vehicle.is_armable:

        time.sleep(1)

        # if program stall more than 20 seconds
        if (a > 20):

            print ("Drone is unarmable, program will quit now.\n")
            # kill the program
            sys.exit()
        
        # increase the time counter
        else:

            time.sleep(1)
            a += 1

    # arm drone and change mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # double check if drone is in armed state or not
    while not vehicle.armed:

        time.sleep(1)

    # take off to height [m]
    print ("Drone will take off now!\n")
    vehicle.simple_takeoff(height)

    # wait until the vehicle reaches a safe height before landing
    while True:

        # take reading from rangefinder
        print("  Height： %.2f [m]" % float(vehicle.rangefinder.distance))
        
        # break and return from function just below target altitude
        if (float(vehicle.rangefinder.distance) >= height * b):
            
            print ("\n%d [m] reached.\n" % height)
            break

        time.sleep(0.5)

    time.sleep(10)

# define moving to specified GPS location sequence
def move_gps_target():

    # specify drone target airspeed
    print ("Set target airspeed to 3 [m/s].\n")
    vehicle.airspeed = 3

    # specify GPS target
    gps_array = [33.959338, 131.191675, 2]
    target = LocationGlobalRelative(gps_array[0], gps_array[1], gps_array[2])
    print ("GPS target x-axis: %s°." % gps_array[0])
    print ("GPS target y-axis: %s°." % gps_array[1])
    print ("GPS target z-axis: %s [m].\n" % gps_array[2])

    # fly to specified GPS target
    vehicle.simple_goto(target)

    time.sleep(10)

# define yaw drone movement sequence
def condition_yaw(heading, relative = False):
    
    if relative:

        # yaw relative to direction of travel
        is_relative = 1

    else:

        # yaw is an absolute angle
        is_relative = 0

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  #command
        0,                                      #confirmation
        heading,                                # param 1, yaw in degrees
        0,                                      # param 2, yaw speed deg/s
        1,                                      # param 3, direction -1 ccw, 1 cw
        is_relative,                            # param 4, relative offset 1, absolute angle 0
        0, 0, 0)                                # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# define x, y, z axis movement sequence
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                      # time_boot_ms (not used)
        0, 0,                                   # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # frame
        0b0000111111000111,                     # type_mask (only speeds enabled)
        0, 0, 0,                                # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,     # x, y, z velocity in [m/s]
        0, 0, 0,                                # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                   # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):

        vehicle.send_mavlink(msg)
        time.sleep(1)

# establishing connection from RPi to PixHawk
vehicle = connect('/dev/ttyS0', heartbeat_timeout = 30, baud = 57600, status_printer = suppress_dronekit_messages)

# initialize take off sequence with height 1.5 [m]
print ("Take off sequence start.\n")
arm_and_takeoff(1.5)

# initialize moving to specified GPS location sequence
print ("Moving to specified GPS location sequence start.\n")
move_gps_target()

# initialize yaw movement drone facing north sequence
print ("Yaw movement drone facing north sequence start.\n")
condition_yaw(0)

# initialize x, y, z axis movement sequence
print ("x, y, z axis movement sequence start.\n")
# movement set 1
print ("Test movement set 1.\n")
send_ned_velocity(   1,   0,   0,   1)
send_ned_velocity(   0,   1,   0,   1)
send_ned_velocity(   0,   0,   1,   1)
# movement set 2
print ("Test movement set 2.\n")
send_ned_velocity(-0.5,   0,   0,   1)
send_ned_velocity(-0.5,   0,   0,   1)
send_ned_velocity(   0,  -1,   0,   1)

# land the drone
print("Change drone mode to LAND.\n")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Experiment completed.\n")
vehicle.close()