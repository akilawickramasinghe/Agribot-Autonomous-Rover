import rover
from simple_pid import PID
import time

USE_PID_YAW = True

MAX_SPEED = 170      # Pulse width with milli seconds
MAX_STEER = 150        # In degrees

P_STEER = 100 #0.02 #orgineel 0.01
I_STEER = 0
D_STEER = 0
steerSetpoint = 1500 # in degrees
angleSetPoint = 30 

steerChannel = 1
speedChannel = 3

control_loop_active = True
pidSteer = None
movementSteerAngle = 0

#These are the input values from the gamepad
inputValueSteer = 0  
inputValueSpeed = 0

control_loop_active = True
flight_altitude = 4

grnd_speed = 1600

def connect_rover(rover_port):
    rover.connect_rover(rover_port) #'/dev/ttyACM0'

def configure_PID(control):
    global pidSteer
    """ Creates a new PID object depending on whether or not the PID or P is used """ 
    print("Configuring control")

    if control == 'PID':
        pidSteer = PID(P_STEER, I_STEER, D_STEER, setpoint=0)       # I = 0.001
        pidSteer.output_limits = (-MAX_STEER, MAX_STEER)          # PID Range
        print("Configuring PID")
    else:
        pidSteer = PID(P_STEER, 0, 0, setpoint=0)               # I = 0.001
        pidSteer.output_limits = (-MAX_STEER, MAX_STEER)          # PID Range
        print("Configuring P")

def setPathdelta(path_delta):
    global inputValueSteer
    inputValueSteer = path_delta

def getMovementSteerAngle():
    return movementSteerAngle

def set_system_state(current_state):
    global state
    state = current_state

def set_flight_altitude(alt):
    global flight_altitude
    flight_altitude = alt
# end control functions

#rover functions
def arm_and_takeoff():
    rover.arm()
    rover.initializeChannelOverride()
    rover.clearAllOverrides()

def disarm():
    rover.disarm()


def print_rover_report():
    print(rover.get_EKF_status())
    print(rover.get_battery_info())
    print(rover.get_version())
#end rover functions

def initialize_debug_logs(DEBUG_FILEPATH):
    global debug_steer
    debug_yaw = open(DEBUG_FILEPATH + "_steer.txt", "a")
    debug_yaw.write("P: I: D: Error: command:\n")

def debug_writer_STEER(value):
    global debug_steer
    debug_steer.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueSteer) + "," + str(value) + "\n")


def control_rover():
    global movementsteerAngle

    if inputValueSteer == 0:
        #rover.overrideChannel(steerChannel,steerSetpoint)
        stop_rover()
    else:
        movementSteerAngle = (pidSteer(inputValueSteer) * -1)
        #rover.send_movement_command_XYA(grnd_speed, movementSteerAngle,flight_altitude)
        rover.overrideChannel(int(steerChannel),int(steerSetpoint-inputValueSteer))#movementSteerAngle))
        rover.overrideChannel(int(speedChannel),1600)
        #debug_writer_STEER(movementSteerAngle)

def stop_rover():
    rover.overrideChannel(speedChannel,steerSetpoint)
    rover.overrideChannel(steerChannel,steerSetpoint)


#Induvidual Module Testing
#connection_string = 'tcp:127.0.0.1:5763'
#connect_rover(connection_string)
#while 1:
#    rover.overrideChannel(1,1600)
    #control_rover()
