class PID_diff:
    def __init__(self, Kp, Ki, Kd, dt, initial_output=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.A0 = Kp + Ki * dt + Kd / dt
        self.A1 = -Kp - 2 * Kd / dt
        self.A2 = Kd / dt

        self.error = [0.0, 0.0, 0.0]

        self.output = initial_output

    def update(self, setpoint, measured_value):
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = setpoint - measured_value

        self.output += self.A0 * self.error[0] + self.A1 * self.error[1] + self.A2 * self.error[2]
        return self.output
