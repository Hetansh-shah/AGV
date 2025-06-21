import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import struct

# Wheel constants
WHEEL_DIAMETER = 0.1397  # meters
ENCODER_RESOLUTION = 4096
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Serial connection setup
        self.motor1_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.motor2_port = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)

        self.motors_enabled = False
        self.previous_left_angle = 0.0
        self.previous_right_angle = 0.0
        self.x_position = 0.0
        self.y_position = 0.0
        self.theta = 0.0

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        if not self.motors_enabled:
            self.clear_errors(self.motor1_port)
            self.clear_errors(self.motor2_port)
            self.set_operation_mode(self.motor1_port, 3)
            self.set_operation_mode(self.motor2_port, 3)
            self.enable_motor(self.motor1_port)
            self.enable_motor(self.motor2_port)
            self.motors_enabled = True
            self.get_logger().info("Motors enabled and initialized.")

        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z

        self.send_motor_speed(self.motor1_port, left_speed)
        self.send_motor_speed(self.motor2_port, right_speed)

        if left_speed != 0 or right_speed != 0:
            self.update_odometry(left_speed, right_speed)

    def send_motor_speed(self, motor_port, speed_rpm):
        speed_data = int(speed_rpm * 65536 / 60)
        data = [0x00, 0x00, (speed_data >> 8) & 0xFF, speed_data & 0xFF]
        self.write_data(motor_port, 0x70B1, data)

    def write_data(self, motor_port, addr, data):
        addr_h = (addr >> 8) & 0xFF
        addr_l = addr & 0xFF
        frame = [0x01, 0x52, addr_h, addr_l, 0x00] + data
        checksum = sum(frame) & 0xFF
        frame.append(checksum)
        motor_port.write(bytes(frame))
        self.get_logger().info(f"Sent command: {frame}")

    def enable_motor(self, motor_port):
        return self.write_data(motor_port, 0x7019, [0x00, 0x00, 0x00, 0x0F])

    def clear_errors(self, motor_port):
        return self.write_data(motor_port, 0x7019, [0x00, 0x00, 0x00, 0x86])

    def set_operation_mode(self, motor_port, mode):
        return self.write_data(motor_port, 0x7017, [0x00, 0x00, 0x00, mode & 0xFF])

    def get_motor_angle(self, motor_port):
        motor_port.write(b'\x01\xA0\x70\x71\x00\x00\x00\x00\x00\xF2')
        response = motor_port.read(10)
        if len(response) == 10:
            position_bytes = response[4:8]
            position = struct.unpack('<i', position_bytes)[0]
            return (position / ENCODER_RESOLUTION) * (2 * math.pi)
        self.get_logger().warn('Failed to read motor position.')
        return 0.0

    def update_odometry(self, left_speed, right_speed):
        left_angle = self.get_motor_angle(self.motor1_port)
        right_angle = self.get_motor_angle(self.motor2_port)

        delta_left = left_angle - self.previous_left_angle
        delta_right = right_angle - self.previous_right_angle

        left_distance = (delta_left / ENCODER_RESOLUTION) * CIRCUMFERENCE
        right_distance = (delta_right / ENCODER_RESOLUTION) * CIRCUMFERENCE

        self.x_position += (left_distance + right_distance) / 2 * math.cos(self.theta)
        self.y_position += (left_distance + right_distance) / 2 * math.sin(self.theta)
        self.theta += (right_distance - left_distance) / WHEEL_DIAMETER

        self.get_logger().info(f"Odometry -> X: {self.x_position:.2f}, Y: {self.y_position:.2f}, Theta: {self.theta:.2f}")

        self.previous_left_angle = left_angle
        self.previous_right_angle = right_angle

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorController()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
