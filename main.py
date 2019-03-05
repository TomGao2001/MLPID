from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from pid_control import PID
import os
import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_COLOR_REFLECTED)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_COLOR_COLOR)

try:
	BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
	BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # Right
	BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # Left
	BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
except IOError as ee:
	print(ee)

# BP.get_sensor retrieves a sensor value.
# BP.PORT_1 specifies that we are looking for the value of sensor port 1.
# BP.get_sensor returns the sensor value (what we want to display).
try:
	BP.get_sensor(BP.PORT_1)
except brickpi3.SensorError:
	print("Configuring light sensor...")
	error = True
	while error:
		time.sleep(0.03)
		try:
			BP.get_sensor(BP.PORT_1)
			error = False
		except brickpi3.SensorError:
			error = True
	print("Configured.")
try:
	BP.get_sensor(BP.PORT_4)
except brickpi3.SensorError:
	print("Configuring color sensor...")
	error = True
	while error:
		time.sleep(0.03)
		try:
			BP.get_sensor(BP.PORT_4)
			error = False
		except brickpi3.SensorError:
			error = True
	print("Configured.")

Mydict = {0:"Kp", 1:"Ki", 2:"Kd"}
COLOR_OFFSET = 50
PID_count = 0

sampling_interval = 0.02
Ki_memorizing_length = 3.0
Ki_info_length = (int) (Ki_memorizing_length / sampling_interval)

base_speed = 30
MyKp = 0.5
MyKi = 0
MyKd = 0.5
MySpeed = base_speed
pid_controller = PID(MyKp, MyKi, MyKd, Ki_info_length)
pid_controller.resetEpochError()
pid_controller.initialize_Ki_info(Ki_info_length)

MotorA_Offset = BP.get_motor_encoder(BP.PORT_A)
MotorD_Offset = BP.get_motor_encoder(BP.PORT_D)
start_flag = False
cur_switch = 0

TOTAL_ERROR = 0.0

out_start_zone = False
start_time = 0
end_time = 0

def printCurrentParameters():
	print("Current parameters: Kp = " + str(pid_controller.Kp)[:10] + "\nKi = " + str(pid_controller.Ki)[:10], "\nKd = " + str(pid_controller.Kd)[:10])

while (True):
	#initialization()
	while not start_flag:
		os.system('clear')

		print("Tuning Mode: " + Mydict[cur_switch])

		if cur_switch == 0:
			pid_controller.Kp = MyKp + (BP.get_motor_encoder(BP.PORT_A) - MotorA_Offset) / 1000
		elif cur_switch == 1:
			pid_controller.Ki = MyKi + (BP.get_motor_encoder(BP.PORT_A) - MotorA_Offset) / 2000
		elif cur_switch == 2:
			pid_controller.Kd = MyKd + (BP.get_motor_encoder(BP.PORT_A) - MotorA_Offset) / 2000
		
		MySpeed = base_speed + (BP.get_motor_encoder(BP.PORT_D) - MotorD_Offset) / 75
		print("Current Max Speed: " + str(MySpeed)[:5])
		
		if BP.get_sensor(BP.PORT_3):
			while BP.get_sensor(BP.PORT_3):
				pass
			MotorA_Offset = BP.get_motor_encoder(BP.PORT_A)
			if cur_switch == 0:
				MyKp = pid_controller.Kp
			elif cur_switch == 1:
				MyKi = pid_controller.Ki
			elif cur_switch == 2:
				MyKd = pid_controller.Kd
			cur_switch = (cur_switch + 1) % 3

		if BP.get_sensor(BP.PORT_2):
			while BP.get_sensor(BP.PORT_2):
				pass
			start_flag = True
		
		printCurrentParameters()
		time.sleep(sampling_interval)

	while not out_start_zone:
		os.system('clear')
		print("waiting...and be patient...")
		BP.set_motor_power(BP.PORT_C, 7)
		BP.set_motor_power(BP.PORT_B, 7)		
		if BP.get_sensor(BP.PORT_4) == 6:
			start_time = time.time()
			out_start_zone = True
			break
	
	os.system('clear')
	
	error = BP.get_sensor(BP.PORT_1) - COLOR_OFFSET
	
	
	if PID_count % pid_controller.epochLength_ == 0:
		pid_controller.evaluate()
		if(pid_controller.needsTraining_):
			pid_controller.backProp()
		
		pid_controller.resetEpochError()
	
	pid_controller.UpdateError(error)
	steer = pid_controller.TotalError()
	
	printCurrentParameters()
	print("needsTraining" + str(pid_controller.needsTraining_))
	print(pid_controller.currentEpochError_)
	print("Current error: " + str(error))
	print("Current steer: " + str(steer))

	BP.set_motor_power(BP.PORT_C, max(0, MySpeed + steer))
	BP.set_motor_power(BP.PORT_B, max(0, MySpeed - steer))
	PID_count += 1

	TOTAL_ERROR += abs(error) * sampling_interval

	if BP.get_sensor(BP.PORT_2):
		BP.reset_all()
		end_time = time.time()
		print("STOPPED MANUALLY")
		break
	
	if BP.get_sensor(BP.PORT_4) == 5 and out_start_zone:
		BP.reset_all()
		end_time = time.time()
		print("STOPPED")
		break

	time.sleep(sampling_interval)

print("TIME ELAPSED: " + str(end_time - start_time)[:5])
print("PID count: " + str(PID_count))
print("SPEED: " + str(MySpeed)[:5])
print("TOTAL ERROR: " + str(TOTAL_ERROR)[:10])