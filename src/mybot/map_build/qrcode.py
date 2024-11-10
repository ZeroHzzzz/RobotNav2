import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import serial

class QRCodeNavigator(Node):
    def __init__(self):
        super().__init__('qr_code_navigator')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 打开串口，假设下位机通过串口发送数据
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.timer = self.create_timer(1.0, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            qr_data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received QR data: {qr_data}')
            self.navigate_to_goal(qr_data)
        
    def navigate_to_goal(self, qr_data):
        # 假设qr_data包含目标位置的坐标，可以解析为 x, y 坐标
        # 例如："x: 1.0, y: 2.0"
        try:
            x, y = self.parse_qr_data(qr_data)
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation.w = 1.0  # 假设固定朝向
            self.publisher_.publish(goal_msg)
            self.get_logger().info(f'Navigating to goal: x={x}, y={y}')
        except Exception as e:
            self.get_logger().error(f'Error parsing QR data: {e}')
        
    def parse_qr_data(self, qr_data):
        # 解析二维码数据，例如 'x:1.0,y:2.0' 的格式
        parts = qr_data.split(',')
        x = float(parts[0].split(':')[1])
        y = float(parts[1].split(':')[1])
        return x, y

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
