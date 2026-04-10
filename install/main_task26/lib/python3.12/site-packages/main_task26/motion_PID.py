import time
import numpy as np


class PID:
    def __init__(self):

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.set_point = 0.0
        self.sampling_time = 0.0

        self.error = 0.0
        self.output_range = [0, 1]
        self.input_range = [0, 1]
        self.last_error = None
        self.last_time = None
        self.error_sum = 0.0
        self.output = 0.0

    def setConstant(self, kp, ki, kd, sampling_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sampling_time = sampling_time

    def setSetPoints(self, setPoint):
        self.set_point = setPoint

    def setRange(self, input_range, output_range):
        self.input_range = input_range
        self.output_range = output_range

    def compute(self, feedback):
        # print("feedback: {}".format(feedback))
        now = time.time()
        if self.last_time is None:
            self.last_time = now
            self.last_error = self.set_point - feedback
            return 0
        dt = now - self.last_time
        if dt < self.sampling_time:
            return self.output
        self.error = self.set_point - feedback
        de = self.error - self.last_error
        self.last_time, self.last_error = now, self.error
        self.error_sum = np.clip(self.error_sum + self.error, self.output_range[0], self.output_range[1])
        p = self.kp * self.error
        i = self.ki * self.error_sum * dt
        d = self.kd * de / dt if dt > 0 else 0
        output = np.clip(p + i + d, self.output_range[0], self.output_range[1])
        self.output = output
        return self.output

    def reset(self):
        self.last_time = None
        self.last_error = None
        self.error_sum = 0
        self.output = 0

    def getError(self):
        return self.error

    def setError(self, newError):
        self.error = newError


class AdaptivePID(PID):
    def __init__(self):
        PID.__init__(self)
        self.beta = 0
        self.last_output = None
        self.last_feedback = None
        self.a = None
        self.b = None
        self.c = None

    def compute(self, feedback):
        now = time.time()
        if self.last_time is None:
            self.last_time = now
            self.last_error = self.set_point - feedback
            self.last_output = 0
            self.last_feedback = feedback
            return 0
        dt = now - self.last_time
        if dt < self.sampling_time:
            return self.output
        error = self.set_point - feedback
        de = error - self.last_error
        self.last_time = now
        self.last_error = error
        self.error_sum += error
        if self.error_sum > self.output_range[1]:
            self.error_sum = self.output_range
        elif self.error_sum < self.output_range[0]:
            self.error_sum = self.output_range[0]
        p = self.kp * error
        i = self.ki * self.error_sum * dt
        d = self.kd * de / dt
        output = p + i + d
        if output > self.output_range[1]:
            output = self.output_range[1]
        elif output < self.output_range[0]:
            output = self.output_range[0]
        self.output = output

        # Adaptive PID Control
        if self.last_output is None:
            self.last_output = self.output
            self.last_feedback = feedback
            self.a = self.beta / (self.beta + dt)
            self.b = self.kp * (self.beta + dt) / (self.beta * dt)
            self.c = self.ki * self.beta / self.sampling_time
            return self.output
        output_dev = self.output - self.last_output
        feedback_dev = feedback - self.last_feedback
        self.a = self.beta / (self.beta + dt)
        self.b = self.kp * (self.beta + dt) / (self.beta * dt)
        self.c = self.ki * self.beta / self.sampling_time
        self.kp += self.a * self.b * output_dev * feedback_dev / feedback_dev ** 2
        self.ki += self.a * self.c * output_dev / feedback_dev
        self.kd += self.a * self.b * self.beta * output_dev / dt
        self.last_output = self.output
        self.last_feedback = feedback
        return self.output


class PIDControl:
    def __init__(self, Kp=1, Kd=0, Ki=0, SetPoints=0, InMin=0, InMax=0, OutMin=0, OutMax=0, Ti=0.001, Td=0.001,
                 DrawPlot=False):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.Ti = Ti
        self.Td = Td
        self.Output = 0
        self.Sp = SetPoints
        # self.SamplingTime = SetSamplingTime
        self.InRangeMin = InMin
        self.InRangeMax = InMax
        self.OutRangeMin = OutMin
        self.OutRangeMax = OutMax
        self.windup_limit = "DISABLE"
        self.windup_crossing = "DISABLE"
        self.isDrawPlotEnable = DrawPlot
        self.error = 0

    def Init(self):
        self.currTime = time.time()
        self.prevTime = self.currTime
        self.error = 0
        self.prevError = 0
        self.sumError = 0
        self.cP = 0
        self.cI = 0
        self.cD = 0
        self.lastCurrTime_Ti = 0
        self.lastCurrTime_Td = 0

    def setEnableWindUpLimit(self):
        self.windup_limit = "ENABLE"

    def setDisableWindUpLimit(self):
        self.windup_limit = "DISABLE"

    def setEnableWindUpCrossing(self):
        self.windup_crossing = "ENABLE"

    def setDisableWindUpCrossing(self):
        self.windup_crossing = "DISABLE"

    def setSetPoints(self, Sp):
        self.Sp = Sp

    def setError(self, Error):
        self.error = Error

    def setTime(self, Ti, Td):
        self.Ti = Ti
        self.Td = Td

    def setRange(self, InMin, InMax, OutMin, OutMax):
        self.InRangeMin = InMin
        self.InRangeMax = InMax
        self.OutRangeMin = OutMin
        self.OutRangeMax = OutMax

    def setConstant(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def getCurrentTime(self):
        return int(self.currTime)

    def getPrevTime(self):
        return int(self.prevTime)

    def getOutput(self):
        return self.OutRangeMax if self.Output > self.OutRangeMax else self.OutRangeMin if self.Output < self.OutRangeMin else self.Output

    def getError(self):
        return self.error

    def calculate(self, FeedBack):
        self.currTime = int(time.time() * 1000)  # S => mS
        deltaTime = self.currTime - self.prevTime  # dt
        deltaError = self.error - self.prevError  # de
        self.error = (self.Sp - FeedBack) / (self.InRangeMax - self.InRangeMin)  #
        self.cP = self.error
        # self.cI += error * deltaTime
        # self.cD = (deltaError / deltaTime) if deltaTime > 0 else 0
        self.cI = self.sumError
        self.cD = deltaError
        self.Output = sum([self.Kp * self.cP,
                           self.Ki * self.cI,
                           self.Kd * self.cD])

        self.Output = self.Output * (self.OutRangeMax - self.OutRangeMin)
        if self.Output > self.OutRangeMax:
            self.Output = self.OutRangeMax
        else:
            if self.Output < self.OutRangeMin:
                self.Output = self.Output
            else:
                self.Output = self.Output
        if self.currTime - self.lastCurrTime_Ti > self.Ti:
            self.lastCurrTime_Ti = self.currTime
            if self.windup_limit == "ENABLE":  # ANTI WINDUP LIMIT - GUARD
                self.sumError = self.sumError + self.error
                if self.sumError > 1.0:
                    self.sumError = 1.0
                else:
                    if self.sumError < -1.0:
                        self.sumError = -1.0
                    else:
                        self.sumError = self.sumError
        if self.currTime - self.lastCurrTime_Td > self.Td:
            self.lastCurrTime_Td = self.currTime
            self.prevError = self.error

        if self.windup_crossing == "ENABLE":  # ANTI WINDUP CROSSING - GUARD
            if self.prevError * self.error < 0:
                self.sumError = 0
        self.prevTime = self.currTime
        self.prevError = self.error
        self.Output = self.Output
        return self.Output

    def reset(self):
        self.prevError = 0
        self.sumError = 0
        self.Output = 0