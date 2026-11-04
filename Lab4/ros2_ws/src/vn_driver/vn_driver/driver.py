"""EECE5554 Lab3 VectorNav VN-100 ROS 2 driver.

Reads checksummed VNYMR sentences on a serial port, configures registers
6 (VNYMR) and 7 (40 Hz), publishes vn_msgs/Vectornav on /imu.
"""

import math

import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from vn_msgs.msg import Vectornav


def checksum(body: str) -> str:
    value = 0
    for byte in body.encode('ascii'):
        value ^= byte
    return f'{value:02X}'


def command(body: str) -> bytes:
    return f'${body}*{checksum(body)}\r\n'.encode('ascii')


def quaternion_from_ypr(yaw_deg, pitch_deg, roll_deg):
    yaw, pitch, roll = map(math.radians, (yaw_deg, pitch_deg, roll_deg))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_vnymr(raw: bytes):
    """Return (values, raw_sentence) only for a valid 12-field VNYMR record."""
    sentence = raw.decode('ascii', errors='replace').strip()
    if not sentence.startswith('$VNYMR,'):
        return None
    if sentence.count('*') != 1:
        return None
    body, given = sentence[1:].split('*', 1)
    if len(given) != 2 or given.upper() != checksum(body):
        return None
    fields = body.split(',')
    if len(fields) != 13 or fields[0] != 'VNYMR':
        return None
    try:
        values = [float(field) for field in fields[1:]]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values, sentence


class VNDriver(Node):
    def __init__(self):
        super().__init__('vn_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        self.publisher = self.create_publisher(Vectornav, '/imu', 100)
        self.serial = Serial(port=port, baudrate=baudrate, timeout=0.2)
        self.get_logger().info(f'Opened {port} at {baudrate} baud')
        # Do not save these settings in nonvolatile memory: shared hardware.
        for body in ('VNWRG,06,14', 'VNWRG,07,40'):
            self.serial.write(command(body))
            self.serial.flush()
            self.get_logger().info(f'Sent configuration: {body}')

    def run(self):
        while rclpy.ok():
            try:
                raw = self.serial.readline()
            except SerialException as error:
                self.get_logger().error(f'Serial read error: {error}')
                break
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            if line.startswith('$VNWRG,'):
                self.get_logger().info(f'Register echo: {line}')
                continue
            if line.startswith('$VNERR,'):
                self.get_logger().error(f'VN-100 command error: {line}')
                continue
            parsed = parse_vnymr(raw)
            if parsed is None:
                continue
            values, original = parsed
            yaw, pitch, roll = values[:3]
            mag_x, mag_y, mag_z = values[3:6]
            accel_x, accel_y, accel_z = values[6:9]
            gyro_x, gyro_y, gyro_z = values[9:12]
            stamp = self.get_clock().now().to_msg()
            msg = Vectornav()
            msg.header.stamp = stamp
            msg.header.frame_id = 'imu1_frame'
            msg.imu.header.stamp = stamp
            msg.imu.header.frame_id = 'imu1_frame'
            msg.mag_field.header.stamp = stamp
            msg.mag_field.header.frame_id = 'imu1_frame'
            q = quaternion_from_ypr(yaw, pitch, roll)
            (msg.imu.orientation.x, msg.imu.orientation.y,
             msg.imu.orientation.z, msg.imu.orientation.w) = q
            msg.imu.angular_velocity.x = gyro_x
            msg.imu.angular_velocity.y = gyro_y
            msg.imu.angular_velocity.z = gyro_z
            msg.imu.linear_acceleration.x = accel_x
            msg.imu.linear_acceleration.y = accel_y
            msg.imu.linear_acceleration.z = accel_z
            msg.mag_field.magnetic_field.x = mag_x * 1e-4
            msg.mag_field.magnetic_field.y = mag_y * 1e-4
            msg.mag_field.magnetic_field.z = mag_z * 1e-4
            msg.vnymr_read = original
            self.publisher.publish(msg)

    def close(self):
        if self.serial.is_open:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VNDriver()
        node.run()
    except (SerialException, OSError) as error:
        if node is not None:
            node.get_logger().error(f'Unable to use serial port: {error}')
        else:
            print(f'Unable to open serial port: {error}')
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

