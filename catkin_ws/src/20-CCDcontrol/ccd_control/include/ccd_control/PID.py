import rospy

class PIDController():
    def __init__(self, PGain, IGian, DGain, SetPoint=0, MaxInt=100):
        self.time = rospy.Time.now()
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
        self.IntWind = self.IntWind + Error * (rospy.Time.now() - self.time)
        if self.IntWind >= self.MaxInt:
            self.IntWind = self.MaxInt

        # Calculate current derivative
        derivative = (Error - self.preError) / (rospy.Time.now() - self.time)

        # Calculate current control
        control = self.PGain * Error + self.IGian * self.IntWind + self.DGain * derivative

        # Update variables
        self.preError = Error
        self.time = rospy.Time.now()

        return control
