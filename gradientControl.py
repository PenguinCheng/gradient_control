#!/usr/bin/python
# coding=utf-8
"""
Gradient control based on relative position to keep two quadrotors mataining a constant distance. 
Use gps position of another quadrotors which is sent by zigbee.  
Communication: point to point
@author: Peng Cheng
		 Hongliang Shen
		 Xi Zhang 
@date: 2015/11/3    09:41    v0.1: the original version.
		
"""
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import time
import serial
import threading
import math

from leaderControl import leaderControl

# function: record currrent time(s)
current_milli_time = lambda: int(time.time() * 1000)

# First get an instance of the API endpoint
api = local_connect()
# Get the connected vehicle (currently only one vehicle can be returned).
vehicle = api.get_vehicles()[0]

# function: takeoff to the target height
def arm_and_takeoff(aTargetAltitude):
	print "Basic pre-arm checks"
	if vehicle.mode.name == "INITIALISING":
		print "Waiting for vehicle to initialise"
		time.sleep(1)
	while vehicle.gps_0.fix_type < 2:
		print "Waiting for GPS........", vehicle.gps_0.fix_type
		time.sleep(1)

	print "Arming motors"

	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	vehicle.flush()

	while not vehicle.armed and not api.exit:
		print "Waiting for arming..."
		time.sleep(1)

	print "Take off!"
	vehicle.commands.takeoff(aTargetAltitude)
	vehicle.flush()

	#Wait until the vehicle reaches a safe height
	while not api.exit:
		print "Altitude: ", vehicle.location.alt
		if vehicle.location.alt >= aTargetAltitude*0.95:
			print "Reached target altitude"
			break;
		time.sleep(1)

# function: controlling vehicle movement using velocity
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
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
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_global_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
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
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


# function: decode the received datas
def decode_position(rec_str):
	if rec_str:
		lonStr = ""
		latStr = ""
		i = 1
		if rec_str[0] == "o":
			while i < len(rec_str):
				if rec_str[i] == "a":
					break
				lonStr += rec_str[i]
				i += 1
			i += 1
			while i < len(rec_str):
				if rec_str[i] == "\n":
					break
				latStr += rec_str[i]
				i +=1

		try:
			lon = float(lonStr)
			lat = float(latStr)
		except Exception, e:
			print "decode exception:", e
			return "", ""
		else:
			return lon, lat
	else:
		return "", ""

# function: the child thread running function, which
#           monitors the serial port all the time 
def readThread(positionList):
	print "child thread begins!"
	receivedDatas = ""
	#global neighbourLon, neighbourLat
	while True:
		receivedDatas = myserial.read(myserial.inWaiting())
		if receivedDatas:
			#print "rec: ", receivedDatas
			positionList[0], positionList[1] = decode_position(receivedDatas)
			#print "neighbourLot: ", positionList[0]
			#print "neighbourLat: ", positionList[1]


# Get Vehicle Home location ((0 index in Vehicle.commands)
print "Get home location" 
cmds = vehicle.commands
cmds.download()
cmds.wait_valid()
print " Home WP: %s" % cmds[0]

# open the serial port between odroid and zibgee
myserial = serial.Serial('/dev/ttyUSB1', 115200, timeout=None)
print myserial.portstr

# open or create a file
read = file('/home/odroid/multiDrone_Com/project_v0.1/velocity/datalog_11_6','a+')
#clear 
read.truncate()

if myserial.isOpen():

    '''initialise datas'''
    delt_T = 100  #500ms
    lastRecord = current_milli_time()
    positionList = [255.0, 255.0]
    loop_cnt = 1
    R = 6371000    #meters
    CONST_VX0 = 0.0
    CONST_VY0 = 0.0
    CONST_D12 = 15.0   #meter
    not_received_flag = 0

    ''' child thread'''
    childThread = threading.Thread(target = readThread, args = (positionList, ))
    #主线程退出时，子线程也退出
    childThread.setDaemon(True)
    childThread.start()
    
    ''' make sure the zigbee conneciton is success before takeing off'''
    while True:
	myserial.write('o'+str("%.8f" % vehicle.location.lon))
     	myserial.write('a'+str("%.8f" % vehicle.location.lat))
	time.sleep(0.5)
	tempos1 = positionList[0]
	tempos2 = positionList[1]
	print "waiting for connecting..."
		
	if tempos1 != 255.0 and tempos2 != 255.0:
		print "connecting ok!"
		break
		
    arm_and_takeoff(3.5)
    
    '''
    After the vehicle reaches a target height, do other things
    '''

    ''' main thread'''
    #leader object 
    leader = leaderControl(CONST_VX0, CONST_VY0, CONST_D12, float(math.pi / 180)*R*vehicle.location.lat, float(math.pi / 180)*R*vehicle.location.lon)  #vx0 = 0.5, vy0 = 0.5, x0 = vehicle.location.lat, y0 = vehicle.location.lon

    '''control loop'''
    while not api.exit:
    	not_received_flag = 0
    	if vehicle.mode.name != "GUIDED":
    		print "User has changed fight mode, aborting loop!"
    		break

    	if current_milli_time() - lastRecord >= delt_T:  #500ms
    		lastRecord = current_milli_time()
    		print "[%s] current time is: %f" % (loop_cnt, lastRecord)
    		loop_cnt += 1

    		#write
    		myserial.write('o'+str("%.8f" % vehicle.location.lon))
    		myserial.write('a'+str("%.8f" % vehicle.location.lat))

    		#read
    		neighbourLon = positionList[0]
    		neighbourLat = positionList[1]

    		print "neighbourLat: ", neighbourLat
    		print "neighbourLon: ", neighbourLon

    		if neighbourLon == "" or neighbourLat == "":      # if receives no datas or decodes incorrecet
    			#lastRecord = current_milli_time() - delt_T - 1   # inmediately go into the next loop
    			not_received_flag = 1
    		else:
    			neighbourLonToMeter = float(math.pi / 180) * R * neighbourLon
    			neighbourLatToMeter = float(math.pi / 180) * R * neighbourLat

    		if not_received_flag == 0:
    			# control
    			leader.x = float(math.pi / 180) * R * vehicle.location.lat
    			leader.y = float(math.pi / 180) * R * vehicle.location.lon
    	    		realV    = vehicle.velocity
    	    		leader.controller(leader.x, neighbourLatToMeter, leader.y, neighbourLonToMeter)

    	    		read.write(str(leader.vx)+" "+str(leader.vy)+" ");
    	    		read.write(str(vehicle.location.lat)+" "+str(vehicle.location.lon)+" ");
    	    		read.write(str(realV[0])+" "+str(realV[1])+" "+str(realV[2])+" ");

    		    	if leader.vx >= 1: #speed protection
    	    			leader.vx = 1
    	    		elif leader.vx <= -1:
    	    			leader.vx = -1
    	    		if leader.vy >= 1:
    	    			leader.vy = 1
    	    		elif leader.vy <= -1:
    	    			leader.vy = -1
				
			read.write(str(leader.vx)+" "+str(leader.vy)+" ");
			read.write(str(current_milli_time())+"\n");
	
			send_ned_velocity(leader.vx, leader.vy, 0);  #vz = 0.0

    '''finished and landing'''
    vehicle.mode = VehicleMode("LAND") 
    read.close	
else:
    print 'serial port do not open!'
    myserial.closeCon()
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")
