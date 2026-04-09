#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

class STM32Driver(Node):
    def __init__(self):
        super().__init__('stm32_driver_node')
        
        # --- 1. 参数配置 ---
        # 声明串口参数，方便运行时修改
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- 2. 初始化串口 ---
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"成功连接串口: {port} 波特率: {baud}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            # 连接失败就不往下走了
            return

        # --- 3. 订阅 /cmd_vel 话题 ---
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info("STM32底盘驱动节点已启动，正在监听 /cmd_vel ...")

    def cmd_vel_callback(self, msg):
        # 1. 获取线速度和角速度
        vx = msg.linear.x
        az = msg.angular.z

        # 为了调试，打印一下收到的数据
        self.get_logger().info(f"收到指令 -> Vx: {vx:.2f}, Az: {az:.2f}")

        # 2. 数据打包 (核心部分)
        # 协议: Head1(0xAA) Head2(0x55) Vx(4B) Az(4B) Sum(1B)
        
        # 使用 struct 库将 float 转为 4字节 (小端模式 <f)
        vx_bytes = struct.pack('<f', vx)
        az_bytes = struct.pack('<f', az)
        
        # 3. 计算校验和 (Sum)
        # 根据你的 C 代码：for (int i = 0; i < 8; i++) checksum += pc_rx_data_buf[i];
        # 也就是只累加 Vx 和 Az 的 8 个字节
        data_payload = vx_bytes + az_bytes
        checksum = sum(data_payload) & 0xFF  # & 0xFF 确保只有低8位

        # 4. 拼接完整的帧
        # b'\xAA\x55' 是帧头
        frame = b'\xAA\x55' + data_payload + struct.pack('B', checksum)

        # 5. 发送串口数据
        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"串口写入错误: {e}")

    def destroy_node(self):
        # 关闭节点时记得关串口
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C，不做额外报错
        node.get_logger().info('键盘中断，正在停止节点...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()