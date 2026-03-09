class FallDetection:
    FALLEN_LEFT = 0
    FALLEN_BACKWARD = 1
    FALLEN_STANDUP = 2
    FALLEN_FORWARD = 3
    FALLEN_RIGHT = 4
    
    def __init__(self, limit_angle=60.0):
        self.limit_angle = limit_angle

    def check_status(self, roll_deg, pitch_deg):
        if pitch_deg > self.limit_angle:
            return self.FALLEN_FORWARD
        elif pitch_deg < -self.limit_angle:
            return self.FALLEN_BACKWARD
        elif roll_deg > self.limit_angle:
            return self.FALLEN_RIGHT 
        elif roll_deg < -self.limit_angle:
            return self.FALLEN_LEFT
        else:
            return self.FALLEN_STANDUP