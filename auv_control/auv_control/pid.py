"""PID with output limits and anti-windup. Pure Python, unit-tested."""


class Pid:
    """PID controller.

    update(error, error_rate, dt) -> output in [out_min, out_max].

    error_rate is passed in rather than differentiated here, so the caller
    can use a measured rate (gyro for heading, filtered depth rate for
    depth). Using the measured rate instead of differencing the error means
    a setpoint step does not produce a derivative spike.

    Anti-windup is conditional integration: the integrator does not move
    while the output is saturated AND the error would push it further into
    saturation. This matters for the forward-only pitch thruster: while the
    vehicle is too deep the thruster just sits at zero, and the integrator
    keeps its trim value instead of winding negative.

    i_limit is the most OUTPUT the I-term may contribute (|ki * integral| <=
    i_limit), in the same units as the output. For depth it must exceed the
    thrust needed to hold depth against positive buoyancy, or the vehicle
    settles shallow of the setpoint.
    """

    def __init__(self, kp, ki, kd, i_limit, out_min=-1.0, out_max=1.0):
        if not out_min < out_max:
            raise ValueError(f'out_min ({out_min}) must be < out_max ({out_max})')
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = abs(i_limit)
        # bound on the integral state that gives that output bound
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
