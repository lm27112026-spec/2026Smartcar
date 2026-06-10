"""
pid.py
"""

class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_limit=0, output_limit=0, d_filter_alpha=0.6):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.d_filter_alpha = d_filter_alpha
        
        self._integral = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._is_saturated = False  # 标记输出是否已达极限
        self._last_output = 0.0

    def compute(self, setpoint, measurement, dt):
        if dt <= 0.001:  # 防止除零异常
            dt = 0.001
            
        error = setpoint - measurement
        
        # 1. 比例项 (P)
        p_out = self.kp * error
        
        # 2. 微分项 (D) - 除以 dt 获取真实变化率，并进行低通滤波
        d_raw = (error - self._prev_error) / dt
        self._d_filtered = self.d_filter_alpha * self._d_filtered + (1 - self.d_filter_alpha) * d_raw
        d_out = self.kd * self._d_filtered
        
        # 3. 积分项 (I) - 条件积分 (Anti-Windup)
        # 只有在未饱和，或者误差方向试图让系统退出饱和状态时，才累加积分
        if not self._is_saturated or (error * self._last_output < 0):
            self._integral += error * dt
            # 内部积分绝对限幅
            if self.integral_limit > 0:
                self._integral = max(-self.integral_limit, min(self._integral, self.integral_limit))
        
        i_out = self.ki * self._integral
        self._prev_error = error
        
        # 4. 计算总输出
        output = p_out + i_out + d_out
        self._last_output = output
        
        # 5. 输出限幅 & 饱和判定
        if self.output_limit > 0:
            if output > self.output_limit:
                output = self.output_limit
                self._is_saturated = True
            elif output < -self.output_limit:
                output = -self.output_limit
                self._is_saturated = True
            else:
                self._is_saturated = False
        else:
            self._is_saturated = False
            
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._is_saturated = False
        self._last_output = 0.0