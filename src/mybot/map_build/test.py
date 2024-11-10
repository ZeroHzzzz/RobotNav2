import serial
import time

def listen_and_execute_command(serial_port):
    """
    持续监听串口数据，一旦接收到任何信息即执行对应的命令。

    参数:
    - serial_port: 已打开的串口对象 (serial.Serial)
    """
    print("开始监听串口数据...")
    cmd = 'FD 04 44 0A 19 00 00 00 CB'
    print(f"Sent: {cmd}")
    serial_port.write(bytes.fromhex(cmd))
    while True:
        # 检查串口缓冲区中是否有数据
        if serial_port.in_waiting > 0:
            # 读取当前缓冲区中的所有字节
            response = serial_port.read(serial_port.in_waiting)
            
            # 将接收到的字节转换成十六进制字符串格式进行打印while True:
            response_hex = ' '.join(f'{byte:02X}' for byte in response)
            print(f"Received data: {response_hex}")
            
            # 执行所需的操作
            execute_command(response)
            break
        
        # 小延时，避免频繁占用 CPU 资源
        time.sleep(0.1)

def execute_command(response):
    """
    根据接收到的响应执行对应操作。
    """
    print("执行命令...")
    # 在此处定义实际操作，例如处理数据或发送响应
    print("操作完成！")
    print(f"Received data: {response}")

# 使用示例
def main():
    # 打开串口
    serial_port = serial.Serial(port="/dev/ttyACM1", baudrate=115200, timeout=0.5)
    if serial_port.is_open:
        # 持续监听并在接收到任意数据时执行命令
        listen_and_execute_command(serial_port)

        # 关闭串口（如果循环在其他地方停止）
        serial_port.close()
    else:
        print("Failed to open serial port.")


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped

# class GoalSubscriber(Node):
#     def __init__(self):
#         super().__init__('goal_subscriber')
#         self.subscription = self.create_subscription(
#             PoseStamped,
#             '/goal_pose',  # 替换成实际的 goal 话题名称
#             self.goal_callback,
#             10)
#         self.subscription  # 防止垃圾回收

#     def goal_callback(self, msg):
#         self.get_logger().info(f'Received Goal Position: x={msg.pose.position.x}, y={msg.pose.position.y}')

# def main(args=None):
#     rclpy.init(args=args)
#     goal_subscriber = GoalSubscriber()
#     rclpy.spin(goal_subscriber)
#     goal_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
