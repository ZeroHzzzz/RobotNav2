import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import numpy as np

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        
        # 订阅 /cmd_vel 主题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # 防止未使用的变量警告
        self.wheel_radius = 37 # 半径
        self.enable = True
        
        if self.enable:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

        self.get_logger().info("Move commander inited!")
    def speed_to_rpm(self, linear_speed):
        # 线速度转换为角速度(m/s -> rad/s)
        angular_velocity = linear_speed * 1000 / self.wheel_radius
        
        # 角速度转换为转速(rad/s -> RPM)
        rpm = (angular_velocity / (2 * np.pi)) * 60
        return rpm

    def get_sign(self, value):
        if value >= 0:
            return '01'  # 正数或零
        else:
            return '00'  # 负数
    
    def cmd_vel_callback(self, msg):
        # 提取线速度和角速度
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        self.get_logger().info(f"x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.angular.z}")

        # 转换为你需要的格式
        linear_x = self.speed_to_rpm(linear_x)  # 线速度转换为转速
        linear_y = self.speed_to_rpm(linear_y)
        angular_z = angular_z * 100
        
        linear_x_hex = hex(int(abs(linear_x)) & 0xFFFF)[2:].zfill(2).upper()
        linear_y_hex = hex(int(abs(linear_y)) & 0xFFFF)[2:].zfill(2).upper()
        angular_z_hex = hex(int(abs(angular_z)) & 0xFFFF)[2:].zfill(2).upper()
        
        cmd = f"FD 01 {self.get_sign(linear_x)} {linear_x_hex} {self.get_sign(linear_y)} {linear_y_hex} {self.get_sign(angular_z)} {angular_z_hex} CB"

        self.get_logger().info(f"Sent to serial: {cmd}")

        if self.enable:
            self.ser.write(bytes.fromhex(cmd))
            

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_listener = CmdVelListener()

    try:
        rclpy.spin(cmd_vel_listener)
    except KeyboardInterrupt:
        pass
    finally:
        if cmd_vel_listener.enable:
            cmd_vel_listener.ser.close()  # 关闭串口
        cmd_vel_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
