import serial
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

def parse_floats(data):
    """从字符串中解析出六个浮点数"""
    try:
        match = re.match(r"start,(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)", data)
        if match:
            float_values = [float(match.group(i)) for i in range(1, 7)]
            return float_values
        else:
            return None
    except ValueError as e:
        print(f"解析浮点数时出错: {e}")
        return None

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 定时器每0.1秒调用一次回调函数

    def timer_callback(self):
        line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
        if line.startswith("start"):
            floats = parse_floats(line)
            if floats:
                self.publish_imu_data(floats)

    def publish_imu_data(self, floats):
        imu_msg = Imu()

        # 填写header信息
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()  # 获取当前时间
        imu_msg.header.frame_id = 'base_footprint'

        # 设置orientation, 此处假设orientation不变，使用默认值
        imu_msg.orientation = Quaternion(x=-3.125322982611716e-07,
                                         y=9.067750376159398e-07,
                                         z=0.7067855972997602,
                                         w=0.7074278192499228)

        imu_msg.orientation_covariance = [0.0] * 9  # 方向的协方差，假设为0

        # 设置angular_velocity
        imu_msg.angular_velocity.x = floats[3]
        imu_msg.angular_velocity.y = floats[4]
        imu_msg.angular_velocity.z = floats[5]

        imu_msg.angular_velocity_covariance = [4.0e-08, 0.0, 0.0,
                                               0.0, 4.0e-08, 0.0,
                                               0.0, 0.0, 4.0e-08]

        # 设置linear_acceleration
        imu_msg.linear_acceleration.x = floats[0]
        imu_msg.linear_acceleration.y = floats[1]
        imu_msg.linear_acceleration.z = floats[2]

        imu_msg.linear_acceleration_covariance = [0.00028900000000000003, 0.0, 0.0,
                                                  0.0, 0.00028900000000000003, 0.0,
                                                  0.0, 0.0, 0.00028900000000000003]

        # 发布Imu消息
        self.publisher_.publish(imu_msg)
        self.get_logger().info(f"发布IMU数据: {imu_msg}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("程序终止")
    finally:
        imu_publisher.serial_port.close()
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
