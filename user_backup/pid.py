"""
pid.py - PID 控制器
【层】控制层
【功能】标准位置式 PID 算法，支持积分限幅、输出限幅
【使用】
  from pid import PID
  pid = PID(kp=1.0, ki=0.0, kd=0.0, integral_limit=500, output_limit=50000)
  output = pid.compute(setpoint, measurement)
  pid.reset()
【参数】
  kp             比例系数
  ki             积分系数
  kd             微分系数
  integral_limit 积分限幅（0=不限幅）
  output_limit   输出限幅（0=不限幅）
"""


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_limit=0, output_limit=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        p_out = self.kp * error
        self._integral += error
        if self.integral_limit > 0:
            self._integral = max(-self.integral_limit, min(self._integral, self.integral_limit))
        i_out = self.ki * self._integral
        d_out = self.kd * (error - self._prev_error)
        self._prev_error = error
        output = p_out + i_out + d_out
        if self.output_limit > 0:
            output = max(-self.output_limit, min(output, self.output_limit))
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

