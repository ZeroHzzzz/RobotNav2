import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import serial
import re
import numpy as np

class NewBot(Node):
    # 用来监听命令信息并发送给下位机
    def __init__(self):
        super().__init__('cmd_vel_listener')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.subscription  # 防止未使用的变量警告
        self.wheel_radius = 40 # 半径
        self.enable = True

        if self.enable:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
            
        self.get_logger().info("Move commander inited!")

    # def command_convert(self, linear_speed):
    #     # 线速度转换为角速度(m/s -> rad/s)
    #     angular_velocity = linear_speed * 1000 / self.wheel_radius
        
    #     # 角速度转换为转速(rad/s -> RPM)
    #     rpm = (angular_velocity / (2 * np.pi)) * 60
    #     return rpm
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 1. 数值处理（保留符号）
        rpm_val = int(linear_x * 1000)
        ang_val = int(angular_z * 100)  # 放大100倍保留精度

        # 2. 数值范围限制（使用有符号16位整数范围）
        rpm_val = max(min(rpm_val, 32767), -32768)
        ang_val = max(min(ang_val, 32767), -32768)

        # 3. 转换为带符号的16位HEX表示
        def to_signed_hex(val):
            """将带符号整数转换为4字符十六进制表示"""
            # 使用补码表示负数
            if val < 0:
                val = (1 << 16) + val  # 转换为无符号等效值
            return format(val, '04X')

        rpm_hex = to_signed_hex(rpm_val)
        ang_hex = to_signed_hex(ang_val)

        # 4. 计算校验和
        cmd_bytes = [
            0xA5, 0x01,
            int(rpm_hex[:2], 16), int(rpm_hex[2:], 16),  # 拆分为高/低字节
            int(ang_hex[:2], 16), int(ang_hex[2:], 16)
        ]
        checksum = sum(cmd_bytes) & 0xFF
        checksum_hex = format(checksum, '02X')

        # 5. 构建完整命令
        cmd = f"A5 02 {rpm_hex[:2]} {rpm_hex[2:]} {ang_hex[:2]} {ang_hex[2:]} {checksum_hex}"

        self.get_logger().info(f"Sent to serial: {cmd}")

        if self.enable:
            self.ser.write(bytes.fromhex(cmd))

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_listener = NewBot()

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
