import serial
import time

def main():
    # 串口配置
    port = '/dev/ttyUSB0'  # 根据实际情况更改
    baudrate = 115200  # 根据设备的波特率设置

    # 初始化串口
    ser = serial.Serial(port, baudrate, timeout=5)
    # time.sleep(2)  # 等待串口稳定

    if ser.is_open:
        print(f"串口 {port} 已打开")

        try:
            # 发送数据
            send_data = "FD 02 00 00 00 00 00 00 CB"
            ser.write(bytes.fromhex(send_data))
            print(f"发送数据: {send_data}")

            # 接收数据
            time.sleep(0.5)  # 等待数据发送完毕
            received_data = ser.read_all()
            received_data= ' '.join(f'{byte:02X}' for byte in received_data)
            print(f"接收数据: {received_data}")

        except Exception as e:
            print(f"通信过程中出错: {e}")

        finally:
            # 关闭串口
            ser.close()
            print(f"串口 {port} 已关闭")

if __name__ == "__main__":
    main()
