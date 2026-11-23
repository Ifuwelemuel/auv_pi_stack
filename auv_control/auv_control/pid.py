"""PID with output limits and anti-windup."""

class Pid:

    def __init__(self,kp,ki,kd,i_limit,out_min= -1.0, out_max= 1.0):
        if not out_min < out_max:
            raise ValueError(f'out_min ({out_min}) must be < out_max ({out_max})')
        self.kp,self.ki,self.kd = kp,ki,kd
        self.i_limit = abs(i_limit)

        self._i_state_max = self.i_limit / abs(ki) if ki else 0.0
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, error_rate, dt):
        trial = self.integral + error * dt
        trial = max(-self._i_state_max, min(self._i_state_max, trial))
        raw = self.kp * error + self.ki * trial + self.kd * error_rate
        if (raw > self.out_max and error > 0.0) or \
           (raw < self.out_min and error < 0.0):
            trial = self.integral                      # freeze
            raw = self.kp * error + self.ki * trial + self.kd * error_rate
        self.integral = trial
        return max(self.out_min, min(self.out_max, raw))


