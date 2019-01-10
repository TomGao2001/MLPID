from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from pid_control import PID

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_COLOR_REFLECTED)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
try:
	BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
	BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # Right
	BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_B))  # Left
except IOError as ee:
	print(ee)

# BP.get_sensor retrieves a sensor value.
# BP.PORT_1 specifies that we are looking for the value of sensor port 1.
# BP.get_sensor returns the sensor value (what we want to display).
try:
	BP.get_sensor(BP.PORT_1)
except brickpi3.SensorError:
	print("Configuring...")
	error = True
	while error:
		time.sleep(0.1)
		try:
			BP.get_sensor(BP.PORT_1)
			error = False
		except brickpi3.SensorError:
			error = True
	print("Configured.")

pid_controller = PID(0.25,0,0.75)
color_offset = 50
#from the lesson
pid_controller.resetEpochError()
PID_count = 0
Lmotor_last_speed = 0
Rmotor_last_speed = 0
touched = False
encoder_offset = BP.get_motor_encoder(BP.PORT_A)
while (True):
	pid_controller.Kd = (BP.get_motor_encoder(BP.PORT_A) - encoder_offset) / 750

	print("Current Kd value: " + str(pid_controller.Kd))
	curr_color_val = BP.get_sensor(BP.PORT_1)
	error = curr_color_val - color_offset
	if error < 5 and error > -5:
		error = 0
	print("Current error: " + str(error))

	try:
		touched = BP.get_sensor(BP.PORT_2)
	except brickpi3.SensorError as eee:
		print(eee)
	if touched:
		BP.reset_all()
		break
	# Offset to absolute center
	'''
	if PID_count % pid_controller.epochLength_ == 0:
		pid_controller.evaluate()
		if(pid_controller.needsTraining_):
			pid_controller.backProp()
		
		pid_controller.resetEpochError()
	'''
	pid_controller.UpdateError(error)
	steer = pid_controller.TotalError()
	print(steer)
	'''
	left_power = Lmotor_last_speed + steer
	right_power = Rmotor_last_speed - steer

	Lmotor_last_speed = left_power
	Rmotor_last_speed = right_power

	BP.set_motor_power(BP.PORT_C, left_power + 10)
	BP.set_motor_power(BP.PORT_B, right_power + 10)
	'''

	BP.set_motor_power(BP.PORT_C, max(0, 10 + steer))
	BP.set_motor_power(BP.PORT_B, max(0, 10 - steer))
	PID_count += 1

	time.sleep(0.02)