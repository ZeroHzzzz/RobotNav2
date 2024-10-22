import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import numpy as np

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        # 初始化串口
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 根据你的设备修改端口和波特率
        
        # 订阅 /cmd_vel 主题
        self.subscription = self.create_subscription(
            Twist,
            # '/cmd_vel_smoothed',
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # 防止未使用的变量警告
        self.wheel_radius = 80  # 轮子的半径（毫米）
    
    def speed_to_rpm(self, linear_speed):
        # 线速度转换为角速度(rad/s)
        angular_velocity = linear_speed / self.wheel_radius
        
        # 角速度(rad/s)转换为转速(RPM)
        rpm = (angular_velocity / (2 * np.pi)) * 60
        return rpm

    def get_sign(self, value):
        # 获取速度的符号，发送符号位
        return '01' if value >= 0 else '00'
    
    def cmd_vel_callback(self, msg):
        # 提取线速度和角速度
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # 转换为你需要的格式
        linear_x_int = self.speed_to_rpm(linear_x * 1000)  # 线速度转换为转速
        linear_y_int = self.speed_to_rpm(linear_y * 1000)
        angular_z_int = angular_z * 100
        
        # 保证转速转换为16位整数的十六进制表示
        linear_x_hex = hex(int(abs(linear_x_int)) & 0xFFFF)[2:].zfill(4).upper()
        linear_y_hex = hex(int(abs(linear_y_int)) & 0xFFFF)[2:].zfill(4).upper()
        angular_z_hex = hex(int(abs(angular_z_int)) & 0xFFFF)[2:].zfill(4).upper()
        
        # 构建数据包，去掉空格以适应 bytes.fromhex
        cmd = f"FD01{self.get_sign(linear_x_int)}{linear_x_hex}{self.get_sign(linear_y_int)}{linear_y_hex}{self.get_sign(angular_z_int)}{angular_z_hex}CB"

        self.get_logger().info(f"Sending command: {cmd}")
        
        # 发送到串口
        try:
            self.ser.write(bytes.fromhex(cmd))
            self.get_logger().info(f"Sent to serial: {cmd}")
        except ValueError as e:
            self.get_logger().error(f"Error converting command to hex: {e}")

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_listener = CmdVelListener()

    try:
        rclpy.spin(cmd_vel_listener)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_listener.ser.close()  # 关闭串口
        cmd_vel_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
