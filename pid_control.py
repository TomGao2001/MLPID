from math import sqrt

class PID(object):
	def __init__(self, Kp, Ki, Kd, iLength):
		self.p_error = 0.0
		self.i_error = 0.0
		self.d_error = 0.0

		self.i_e_fabs = 0.0
		
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

		self.Ki_info_length = iLength

		self.i_info = []
		self.i_pointer = 0

		self.epochLength_ = 10

		self.counter_ = 0

		self.speed_coefficient = 1.0

		self.epochCumulativeError_ = 0.0
		self.previousEpochError_ = 0.0
		self.needsTraining_ = True
		self.currentEpochError_ = 0.0
		self.errorThreshold_ = 0.1
		self.learnRate_ = 0.005

		self.PID_count = 0
	def initialize_Ki_info(self, length):
		for i in range(0,length):
			self.i_info.append(0)

	def updateEpochError(self, cte):
		self.epochCumulativeError_ += (cte*cte)	

	def updateSpeedCoeff(self):
		if self.PID_count > 1000 and abs(self.i_error) < 600:
			self.speed_coefficient = 1.3
		else:
			self.speed_coefficient = 1

	def resetEpochError(self):
		self.epochCumulativeError_ = 0.0

	def evaluate(self):
		self.currentEpochError_ = sqrt(self.epochCumulativeError_ / self.epochLength_)/100
		self.updateSpeedCoeff()
		#self.needsTraining_ = self.currentEpochError_ > self.errorThreshold_	
		self.needsTraining_ = True

	def adjust(self, Kx, dx, dE):
		if Kx == 'p':
			partialDKx = self.Kp * dx * dE * self.learnRate_
			self.Kp -= partialDKx
			self.Kp = max(0, self.Kp)
		if Kx == 'i':
			partialDKx = self.Ki * dx * dE * self.learnRate_
			self.Ki -= partialDKx
			self.Ki = max(0, self.Ki)
		if Kx == 'd':
			partialDKx = self.Kd * dx * dE * self.learnRate_
			self.Kd -= partialDKx
			self.Kd = max(0, self.Kd)

	def backProp(self):
		deltaError = self.previousEpochError_ - self.currentEpochError_
		self.previousEpochError_ = self.currentEpochError_
		self.adjust('p', -self.p_error, deltaError)
		self.adjust('i', -self.i_e_fabs, deltaError)
		self.adjust('d', -self.d_error, deltaError)

	def UpdateError(self, cte):
		self.d_error = cte - self.p_error
		self.p_error = cte
		self.UpdateKiError(cte)#here
		self.updateEpochError(cte)

	def UpdateKiError(self, cte):
		self.i_error-=self.i_info[self.i_pointer]
		self.i_e_fabs-=abs(self.i_info[self.i_pointer])
		self.i_info[self.i_pointer] = cte
		self.i_error+=cte
		self.i_e_fabs+=abs(cte)
		self.i_pointer+=1
		if self.i_pointer == self.Ki_info_length:
			self.i_pointer = 0

	def TotalError(self):
		return -self.Kp * self.p_error - self.Ki * self.i_error - self.Kd * self.d_error