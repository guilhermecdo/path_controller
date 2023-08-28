class PID:
	def __init__(self, 
					kp_linear = 0.3, kd_linear = 0.1, ki_linear = 0, 
					kp_angular = 0.3, kd_angular = 0.1, ki_angular = 0.0):
                
		self.__kp_linear = kp_linear
		self.__kd_linear = kd_linear
		self.__ki_linear = ki_linear

		self.__kp_angular = kp_angular
		self.__kd_angular = kd_angular
		self.__ki_angular = ki_angular

		self.__prev_error_position = 0
		self.__prev_error_angle = 0


	def computeOutput(self, distance_to_target, angle_to_target):
		error_position = distance_to_target
		
		error_angle = angle_to_target

		linear_velocity_control = self.__kp_linear*error_position + self.__kd_linear*(error_position - self.__prev_error_position)
		angular_velocity_control = self.__kp_angular*error_angle + self.__kd_angular*(error_angle - self.__prev_error_angle)

		self.prev_error_angle = error_angle
		self.prev_error_position = error_position

		return linear_velocity_control, angular_velocity_control