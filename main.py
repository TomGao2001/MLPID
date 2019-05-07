from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from pid_control import PID
import matplotlib
import matplotlib.pyplot as plt
import os
import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_COLOR_REFLECTED)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
#BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_COLOR_COLOR)

try:
	#BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
	BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # Right
	BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # Left
	#BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
except IOError as ee:
	print(ee)
'''
ans = raw_input("Use data from last run? (y/n)")
if ans == "y" and os.path.isfile('param.txt'):
	with open("param.txt") as file:
		MyKp = float(file.readline())
		MyKi = float(file.readline())
		MyKd = float(file.readline())
else:
	MyKp = 0.8555
	MyKi = 0.1#0.04
	MyKd = 2
	print("Default values used")
'''	
MyKp = 0.8555
MyKi = 0.1#0.04
MyKd = 2

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

sampling_interval = 0
#500? tbd it actually depends.
Ki_info_length = 100

base_speed = 40

MySpeed = base_speed
pid_controller = PID(MyKp, MyKi, MyKd, Ki_info_length)
pid_controller.resetEpochError()
pid_controller.initialize_Ki_info(Ki_info_length)

#MotorA_Offset = BP.get_motor_encoder(BP.PORT_A)
#MotorD_Offset = BP.get_motor_encoder(BP.PORT_D)
start_flag = False
cur_switch = 0

TOTAL_ERROR = 0.0

out_start_zone = False
start_time = 0
end_time = 0

Kp_history = []
Ki_history = []
Kd_history = []
T = []
def printCurrentParameters():
	print("Current parameters:\nKp = " + str(pid_controller.Kp)[:10] + "\nKi = " + str(pid_controller.Ki)[:10], "\nKd = " + str(pid_controller.Kd)[:10] + "\n")

def printCurrentLearningParameters():
	print("Current parameters:\nKi_info_length : " + str(pid_controller.Ki_info_length) + "\nLearning rate : " + str(pid_controller.learnRate_)[:10])

while (True):
	#initialization()
	while not start_flag:
		os.system('clear')
		print("VOLTAGE:", BP.get_voltage_battery())
		print("Tuning Mode: " + Mydict[cur_switch])
		'''
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
		'''
		if BP.get_sensor(BP.PORT_2):
			while BP.get_sensor(BP.PORT_2):
				pass
			start_flag = True
		
		printCurrentParameters()
		printCurrentLearningParameters()
		time.sleep(sampling_interval)

	while not out_start_zone:
		os.system('clear')
		print("waiting...")
		BP.set_motor_power(BP.PORT_C, 10)
		BP.set_motor_power(BP.PORT_B, 10)		
		if BP.get_sensor(BP.PORT_4) == 6:
			start_time = time.time()
			out_start_zone = True
			break
	
	os.system('clear')
	
	error = BP.get_sensor(BP.PORT_1) - COLOR_OFFSET
	error = min(40,error)
	error = max(-40,error)


	if PID_count % pid_controller.epochLength_ == 0:
		pid_controller.evaluate()
		if(pid_controller.needsTraining_):
			pid_controller.backProp()
		pid_controller.resetEpochError()
	
	pid_controller.UpdateError(error)
	steer = pid_controller.TotalError()
	
	printCurrentParameters()
	print("needsTraining: " + str(pid_controller.needsTraining_))
	print("Epoch error: " + str(pid_controller.epochCumulativeError_) + " " + str(pid_controller.currentEpochError_))
	print("Current cte: " + str(error))
	print("Current steer: " + str(steer))
	print("Current i_error: " + str(pid_controller.i_error))
	print("Current i_e_fabs: "+ str(pid_controller.i_e_fabs))
	print("Current Speed Coefficient: " + str(pid_controller.speed_coefficient))
	BP.set_motor_power(BP.PORT_C, pid_controller.speed_coefficient*min(100,max(0, MySpeed - steer)))
	BP.set_motor_power(BP.PORT_B, pid_controller.speed_coefficient*min(100,max(0, MySpeed + steer)))
	PID_count += 1

	TOTAL_ERROR += abs(error) * (0.01+sampling_interval)

	if BP.get_sensor(BP.PORT_2):
		BP.reset_all()
		end_time = time.time()
		print("STOPPED MANUALLY")
		break

	'''
	if BP.get_sensor(BP.PORT_4) == 5 and out_start_zone:
		BP.reset_all()
		end_time = time.time()
		print("STOPPED")
		break
	'''
	T.append(PID_count)
	Kp_history.append(pid_controller.Kp)
	Ki_history.append(pid_controller.Ki)
	Kd_history.append(pid_controller.Kd)

	time.sleep(sampling_interval)

with open("param.txt","w+") as f:
	f.write(str(pid_controller.Kp)+"\n")
	f.write(str(pid_controller.Ki)+"\n")
	f.write(str(pid_controller.Kd)+"\n")

print("TIME ELAPSED: " + str(end_time - start_time)[:5])
print("PID count: " + str(PID_count))
print("SPEED: " + str(MySpeed)[:5])
print("TOTAL ERROR: " + str(TOTAL_ERROR)[:10])

matplotlib.use('Agg')

fig, (ax0, ax1,ax2) = plt.subplots(nrows = 3, ncols=1, constrained_layout=True)

ax0.plot(T,Kp_history)
ax0.set(xlabel='PID count', ylabel='Kp')
ax0.grid()
ax1.plot(T,Ki_history)
ax1.set(xlabel='PID count', ylabel='Ki')
ax1.grid()
ax2.plot(T,Kd_history)
ax2.set(xlabel='PID count', ylabel='Kd')
ax2.grid()

fig.savefig("pid.png")
plt.show()