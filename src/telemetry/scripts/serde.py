from struct import Struct


class JoyInput:
	AXIS_STEPS = 20
	
	def __init__(self):
		self.axes = [0, 0, 0, 1, 1, 0, 0, 0]
		self.buttons = [False] * 10
	
	def _update_joy(self, idx, axis_value):
		if idx == 0:
			self.axes[0] = axis_value
		elif idx == 1:
			self.axes[1] = - axis_value
		elif idx == 2:
			self.axes[2] = axis_value
		elif idx == 3:
			self.axes[3] = 1 - axis_value * 2
		elif idx == 4:
			self.axes[4] = 1 - axis_value * 2
		elif idx == 5:
			self.axes[5] = - axis_value
	
	def _update_button(self, idx, pressed):
		if idx < 4:
			if pressed:
				if idx == 0:
					self.axes[6] = 1
				elif idx == 1:
					self.axes[6] = -1
				elif idx == 2:
					self.axes[7] = 1
				else:
					self.axes[7] = -1
			else:
				if idx == 0:
					self.axes[6] = 0
				elif idx == 1:
					self.axes[6] = 0
				elif idx == 2:
					self.axes[7] = 0
				else:
					self.axes[7] = 0
		else:
			self.buttons[idx - 4] = pressed
	
	def deserialize_joy_axis(self, byte):
		axis = 0
		if byte >= 128:
			axis += 4
			byte -= 128
		if byte >= 64:
			axis += 2
			byte -= 64
		if byte >= 32:
			axis += 1
			byte -= 32
		if byte > self.AXIS_STEPS:
			raise ValueError("Got byte bigger than:", self.AXIS_STEPS)
		axis_value = float(byte) / self.AXIS_STEPS
		if axis == 3 or axis == 4:
			self._update_button(axis + 3, axis_value >= 0.9)
		else:
			axis_value = axis_value * 2 - 1
		self._update_joy(axis, axis_value)
	
	def deserialize_joy_button(self, byte):
		pressed = False
		if byte >= 128:
			pressed = True
			byte -= 128
		self._update_button(byte, pressed)


def serialize_odometry(odom):
	return serialize_f32([
		odom.pose.pose.orientation[0],
		odom.pose.pose.orientation[1],
		odom.pose.pose.orientation[2],
		odom.pose.pose.orientation[3],
		odom.pose.pose.position[0],
		odom.pose.pose.position[1],
		odom.pose.pose.position[2],
		0,0,0,0,0,0
	])


_i32_struct = Struct("<i")
_f32_struct = Struct("<f")
_f64_struct = Struct("<d")
# The following serialize methods can accept a tuple of numbers; All the numbers will be serialized contiguously
serialize_i32 = _i32_struct.pack
serialize_f32 = _f32_struct.pack
serialize_f64 = _f64_struct.pack

# IMPORTANT NOTE
# The following deserialize methods attempt to deserialize as much as they can from the byte stream
# So they all return tuples
deserialize_i32 = _i32_struct.unpack
deserialize_f32 = _f32_struct.unpack
deserialize_f64 = _f64_struct.unpack
