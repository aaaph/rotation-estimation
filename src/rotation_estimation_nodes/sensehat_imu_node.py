# src/rotation_estimation_nodes/sensehat_udp_imu_node.py
import json
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665


class SenseHatUdpImuNode(Node):
    def __init__(self):
        super().__init__("sensehat_udp_imu_node")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("frame_id", "sensehat_link")
        self.declare_parameter("topic", "/sensehat/imu_raw")

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value

        self.pub = self.create_publisher(Imu, topic, qos_profile_sensor_data)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"Listening UDP {host}:{port}, publishing {topic}")

    def poll(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return

            packet = json.loads(data.decode("utf-8"))
            gyro = packet["gyro"]
            accel = packet["accel"]

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = float(gyro["x"])
            msg.angular_velocity.y = float(gyro["y"])
            msg.angular_velocity.z = float(gyro["z"])

            msg.linear_acceleration.x = float(accel["x"]) * G
            msg.linear_acceleration.y = float(accel["y"]) * G
            msg.linear_acceleration.z = float(accel["z"]) * G

            self.pub.publish(msg)


def main():
    rclpy.init()
    node = SenseHatUdpImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()