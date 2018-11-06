from math import sqrt


class PID(object):
    def __init__(self, Kp, Ki, Kd):
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        self.i_e_fabs = 0.0

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.counter_ = 0
        self.epochLength_ = 200
        self.epochCumulativeError_ = 0.0
        self.previousEpochError_ = 0.0
        self.needsTraining_ = True
        self.currentEpochError_ = 0.0
        self.errorThreshold_ = 0.005
        self.learnRate_ = 0.01

    def updateEpochError(self, cte):
        self.i_e_fabs_ += abs(cte)
        self.epochCumulativeError_ += (cte * cte)

    def resetEpochError(self):
        self.i_e_fabs_ = 0.0
        self.epochCumulativeError_ = 0.0

    def evaluate(self):
        if self.needsTraining_:
            self.currentEpochError_ = sqrt(self.epochCumulativeError_ / self.epochLength_)
            self.needsTraining_ = self.currentEpochError_ > self.errorThreshold_

    def adjust(self, Kx, dx, dE):
        if (Kx == 'p'):
            partialDKx = self.Kp * dx * dE * self.learnRate_
            self.Kp -= partialDKx
        if (Kx == 'i'):
            partialDKx = self.Ki * dx * dE * self.learnRate_
            self.Ki -= partialDKx
        if (Kx == 'd'):
            partialDKx = self.Kd * dx * dE * self.learnRate_
            self.Kd -= partialDKx

    def backProp(self):
        deltaError = self.previousEpochError_ - self.currentEpochError_
        self.previousEpochError_ = self.currentEpochError_
        self.adjust('p', -self.p_error, deltaError)
        self.adjust('i', -self.i_e_fabs_, deltaError)
        self.adjust('d', -self.d_error, deltaError)

    def UpdateError(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        self.updateEpochError(cte)

    def TotalError(self):
        return -self.Kp * self.p_error - self.Ki * self.i_error - self.Kd * self.d_error
