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
		axis_bits = 0
		if byte >= 128:
			axis_bits += 128
			byte -= 128
		if byte >= 64:
			axis_bits += 64
			byte -= 64
		if byte >= 32:
			axis_bits += 32
			byte -= 32
		self._update_joy(axis_bits / 32, byte / self.AXIS_STEPS)
	
	def deserialize_joy_button(self, byte):
		pressed = False
		if byte >= 128:
			pressed = True
			byte -= 128
		self._update_button(byte, pressed)


def serialize_odometry(odom):
	return serialize_f32([
		odom.pose.orientation.x,
		odom.pose.orientation.y,
		odom.pose.orientation.z,
		odom.pose.orientation.w,
		odom.pose.position.x,
		odom.pose.position.y,
		odom.pose.position.z,
		odom.twist.linear.x,
		odom.twist.linear.y,
		odom.twist.linear.z,
		odom.twist.angular.x,
		odom.twist.angular.y,
		odom.twist.angular.z,
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


class DeserializationStream(object):
	"""
	A helper class for deserializing byte arrays with more than one serialized element
	Every deserialize call removes data from the beginning of the byte array
	"""
	
	def __init__(self, data):
		self.data = data
	
	def remaining_bytes(self):
		return len(self.data)
	
	def deserialize_i32(self, count=1):
		"""
		:param count: Number of ints to deserialize at once
		:return: A tuple of ints
		"""
		out = []
		for i in range(count):
			out.append(deserialize_i32(self.data[i * 4: (i + 1) * 4])[0])
			del self.data[i * 4: (i + 1) * 4]
		return out
	
	def deserialize_f32(self, count=1):
		"""
		:param count: Number of floats to deserialize at once
		:return: A tuple of floats
		"""
		out = []
		for i in range(count):
			out.append(deserialize_f32(self.data[i * 4: (i + 1) * 4])[0])
		del self.data[0: count * 4]
		return out
	
	def deserialize_f64(self, count=1):
		"""
		:param count: Number of doubles to deserialize at once
		:return: A tuple of doubles
		"""
		out = []
		for i in range(count):
			out.append(deserialize_f64(self.data[i * 8: (i + 1) * 8])[0])
			del self.data[i * 8: (i + 1) * 8]
		return out