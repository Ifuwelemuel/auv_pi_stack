"""What is fitted on each output, and the pulse each one gets (D19).

Pure Python, no ROS, unit-tested in test/test_actuators.py.

An output is one ESC or one servo, on a Pixhawk AUX pin (stage 'pixhawk')
or a PCA9685 channel (stage 'pca9685'). Each follows ONE of the three
normalised commands in ActuatorCommand ('surge', 'pitch', 'yaw', all in
[-1, 1]), so the rest of the stack does not know whether yaw is a thruster
or a pair of rudders:

    three_thruster   surge, pitch, yaw ESCs          Pixhawk AUX3, AUX2, AUX1
    torpedo (+ fins) surge ESC                       Pixhawk AUX3
                     top + bottom fins <- yaw        PCA9685 ch0, ch1
                     left + right fins <- pitch      PCA9685 ch2, ch3

Each output has three pulses:
    pulse(cmd)   what it gets for a command
    neutral      what it gets when stopped / e-stopped (ESC stop, fin centre)
    failsafe     Pixhawk outputs only: what the Pixhawk must hold when the Pi
                 is silent (AUVW_STOPn); the driver checks this at start-up.
                 A PCA9685 output has no watchdog: it holds its last pulse.
"""
from auv_drivers.esc import thrust_to_us
from auv_drivers.rc_override import NUM_AUX

AXES = ('surge', 'pitch', 'yaw')
KINDS = ('esc', 'servo')
STAGES = ('pixhawk', 'pca9685')


def servo_to_us(cmd, center_us, span_us, sign, min_us, max_us):
    """Normalised deflection [-1, 1] -> pulse width for one servo.

    center_us  pulse with the fin straight (measured trim)
    span_us    pulse change at full command
    sign       +1 or -1, set so the fin moves the right way (see README)
    min_us, max_us  mechanical limits: the result never leaves them, so a
               fin can never be driven into its end stop, whatever the
               span. NaN maps to centre.
    """
    if cmd != cmd:
        return int(center_us)
    x = max(-1.0, min(1.0, float(cmd))) * sign
    return int(round(min(max_us, max(min_us, center_us + x * span_us))))


def pulse(cfg, cmd):
    if cfg['kind'] == 'servo':
        return servo_to_us(cmd, cfg['center_us'], cfg['span_us'], cfg['sign'],
                           cfg['min_us'], cfg['max_us'])
    return thrust_to_us(cmd, cfg['stop_us'], cfg['span_us'], cfg['sign'],
                        cfg['bidirectional'])


def neutral(cfg):
    return int(cfg['center_us'] if cfg['kind'] == 'servo' else cfg['stop_us'])


def failsafe(cfg):
    return int(cfg['failsafe_us'] if cfg['kind'] == 'servo' else cfg['stop_us'])


def describe(cfg):
    if cfg['kind'] == 'servo':
        return (f"servo<-{cfg['input']} centre={cfg['center_us']} span={cfg['span_us']} "
                f"sign={cfg['sign']:+d} limits={cfg['min_us']}..{cfg['max_us']} "
                f"failsafe={cfg['failsafe_us']}")
    return (f"esc<-{cfg['input']} stop={cfg['stop_us']} span={cfg['span_us']} "
            f"sign={cfg['sign']:+d} {'bi' if cfg['bidirectional'] else 'FWD-ONLY'}")


def validate(outs):
    """Raise ValueError naming the parameter to fix. outs: name -> cfg."""
    if not outs:
        raise ValueError('outputs is empty: nothing to drive')
    for n, c in outs.items():
        if c['stage'] not in STAGES:
            raise ValueError(f"{n}_stage = '{c['stage']}': must be 'pixhawk' or 'pca9685'")
        if c['kind'] not in KINDS:
            raise ValueError(f"{n}_kind = '{c['kind']}': must be 'esc' or 'servo'")
        if c['input'] not in AXES:
            raise ValueError(f"{n}_input = '{c['input']}': must be one of {AXES}")
        if c['sign'] not in (1, -1):
            raise ValueError(f"{n}_sign = {c['sign']}: must be 1 or -1")
        if c['kind'] == 'servo':
            if not c['min_us'] <= c['center_us'] <= c['max_us']:
                raise ValueError(f'{n}: need {n}_min_us <= {n}_center_us <= {n}_max_us')
            if not c['min_us'] <= c['failsafe_us'] <= c['max_us']:
                raise ValueError(f'{n}_failsafe_us must lie within {n}_min_us..{n}_max_us')
        if c['stage'] == 'pixhawk' and not 1 <= c['aux'] <= NUM_AUX:
            raise ValueError(f"{n}_aux = {c['aux']}: must be 1..{NUM_AUX} "
                             f"(auv_watchdog.lua drives AUX1-AUX{NUM_AUX})")
        if c['stage'] == 'pca9685' and not 0 <= c['channel'] <= 15:
            raise ValueError(f"{n}_channel = {c['channel']}: must be 0..15")
    for stage, key in (('pixhawk', 'aux'), ('pca9685', 'channel')):
        on = {n: c[key] for n, c in outs.items() if c['stage'] == stage}
        if len(set(on.values())) != len(on):
            raise ValueError(f'two {stage} outputs share a {key}: '
                             + ', '.join(f'{n}={v}' for n, v in on.items()))
    if not any(c['input'] == 'surge' and c['kind'] == 'esc' for c in outs.values()):
        raise ValueError('no ESC follows surge: the vehicle could not move')
