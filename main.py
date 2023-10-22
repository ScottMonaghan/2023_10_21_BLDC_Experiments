import board
import pwmio
from time import (
    sleep,
    monotonic, 
    )
import countio

#modified from https://www.kevsrobots.com/resources/how_it_works/pid-controllers.html
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else: 
            derivative = 0
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


SLEEP_DURATION = 0.0001
PULSES_PER_ROTATION = 45
MINIUM_SAMPLE_DURATION = 0.05

class PulseRecord:
    def __init__(self, pulse_count, timestamp):
        self.pulse_count = pulse_count
        self.timestamp = timestamp

class ZS_X11H_BLDCWheel:
    def __init__(
            self,
            pwm_out:pwmio.PWMOut,
            pulse_counter:countio.Counter,
            pulses_per_rotation:int,
            pid:PID,
            ):
        self.pwm_out = pwm_out
        self.pulse_counter = pulse_counter
        self.pulses_per_rotation = pulses_per_rotation
        self.pid = pid
        self._target_rpm = 0
        self._rpm_samples = []
        self._average_rpm = 0
        self._start_time = 0
        self._last_spot_rpm = 0

    @staticmethod
    def clamp(value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value: 
            return max_value
        else:
            return value
        
    def get_spot_rpm(self) -> float:
        rpm_samples = []
        self.pulse_counter.reset()
        first_timed_pulse = PulseRecord(self.pulse_counter.count, monotonic())
        while self.pulse_counter.count == first_timed_pulse.pulse_count or monotonic() == first_timed_pulse.timestamp:
            sleep(SLEEP_DURATION)
        second_timed_pulse = PulseRecord(self.pulse_counter.count, monotonic())
        while self.pulse_counter.count == second_timed_pulse.pulse_count or monotonic() == second_timed_pulse.timestamp:
            sleep(SLEEP_DURATION)
        third_timed_pulse = PulseRecord(self.pulse_counter.count, monotonic())

        #now we can figure out the speed in pulses per second!
        first_pulses_per_second = (
            (second_timed_pulse.pulse_count - first_timed_pulse.pulse_count)
            / (second_timed_pulse.timestamp - first_timed_pulse.timestamp)
            )
        second_pulses_per_second = (
            (third_timed_pulse.pulse_count - second_timed_pulse.pulse_count)
            / (third_timed_pulse.timestamp - second_timed_pulse.timestamp)
            )
        
        avg_pulses_per_second = (first_pulses_per_second + second_pulses_per_second)/2

        rpm = (avg_pulses_per_second / self.pulses_per_rotation) * 60

        self._rpm_samples.append(rpm)
        if len(self._rpm_samples) > 10:
            self._rpm_samples = self._rpm_samples[-10:]

        self._last_spot_rpm = rpm

        return rpm

    def set_target_rpm(self, target_rpm:int):
        self._target_rpm = target_rpm
        self._rpm_samples.clear()

    def get_target_rpm(self):
        return self._target_rpm

    def get_last_spot_rpm(self):
        return self._last_spot_rpm

    def get_average_rpm(self):
        return sum(self._rpm_samples)/len(self._rpm_samples) if len(self._rpm_samples) > 0 else 0

    def loop(self):
        rpm = self.get_spot_rpm()
        loop_timestamp = monotonic()
        if self._start_time==0:
            self._start_time = loop_timestamp
        dt = loop_timestamp - self._start_time
        error = self._target_rpm * 10 - rpm * 10
        speed = self.pwm_out.duty_cycle
        speed = ZS_X11H_BLDCWheel.clamp(speed+int(self.pid.update(error,dt)),100,65535)
        self.pwm_out.duty_cycle = speed
        sleep(SLEEP_DURATION)
        

right_wheel_pwm = pwmio.PWMOut(board.GP1, frequency=20000)
right_wheel_pulse_counter = countio.Counter(board.GP3, edge=countio.Edge.RISE)
right_wheel_pid = PID(Kp=1.0, Ki=0.0001, Kd=0)

left_wheel_pwm = pwmio.PWMOut(board.GP4, frequency=20000)
left_wheel_pulse_counter = countio.Counter(board.GP7)

right_wheel = ZS_X11H_BLDCWheel(right_wheel_pwm, right_wheel_pulse_counter, PULSES_PER_ROTATION, right_wheel_pid)
speed=100
right_wheel.pwm_out.duty_cycle = speed
error_sum = 0 
#start_time = 0
rpm_samples = []
speed_timer = monotonic()
rpm_timer = monotonic()
multiplier = 1
right_wheel.set_target_rpm(10)
while True:
    right_wheel.loop()
    loop_time = monotonic()

    if loop_time-speed_timer > 6:
        speed_timer = loop_time
        target_rpm = right_wheel.get_target_rpm()
        if target_rpm >= 160:
            multiplier = -1
        elif target_rpm <= 10:
            multiplier = 1
        right_wheel.set_target_rpm(target_rpm + 50 * multiplier)

    if loop_time - rpm_timer > 0.5:
        rpm_timer = loop_time
        print(int(right_wheel.get_average_rpm()))



