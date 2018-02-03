import rospy
import time
class PIDController():
    def __init__(self, PGain, IGian, DGain, SetPoint=0, MaxInt=100):
        self.time = time.time()
        self.preError = 0
        self.IntWind = 0
        self.MaxInt = MaxInt
        self.PGain = PGain
        self.IGian = IGian
        self.DGain = DGain
        self.SetPoint = SetPoint

    def update(self, current):
        # Calculate current error
        Error = self.SetPoint - current

        # Calculate current integral
        current_time = time.time()
        dt = float(current_time - self.time)
        self.IntWind = self.IntWind + Error * dt
        if self.IntWind >= self.MaxInt:
            self.IntWind = self.MaxInt

        # Calculate current derivative
        derivative = (Error - self.preError) / dt

        # Calculate current control
        control = self.PGain * Error + self.IGian * self.IntWind + self.DGain * derivative

        # Update variables
        self.preError = Error
        self.time = time.time()

        return control
