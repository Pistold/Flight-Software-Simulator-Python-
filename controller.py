class PIDController:
    def __init__(self, kp=2.0, ki=0.1, kd=1.5,
                 integral_limit=50.0, output_limit=150.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self._first_step = True

    def compute(self, target, measured, dt):
        error = target - measured

        # Integral with anti-windup clamp
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        # Derivative — skip first step to avoid spike from prev_error=0
        if self._first_step:
            derivative = 0.0
            self._first_step = False
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Clamp total output
        return max(0.0, min(self.output_limit, output))

    def reset(self):
        """Call this if you change targets mid-flight."""
        self.integral = 0.0
        self.prev_error = 0.0
        self._first_step = True