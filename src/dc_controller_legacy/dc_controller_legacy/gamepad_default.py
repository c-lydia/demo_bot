import rclpy
from rclpy.node import Node
from custom_messages.msg import BetterGamePad
from std_msgs.msg import Bool
from custom_messages.msg import SocketHeartbeat
import socket       
import threading 
from threading import Lock
import time
import netifaces

class gamepad_default_legacy_node(Node):
    def __init__(self):
        super().__init__('gamepad_default_legacy')
        self._init_variables()
        self._init_sockets()
        self._init_pub_sub_msg()
        self._init_threads()
        
        self.get_logger().info(f"[RUNNING] Gamepad node started on address: {self.ip}:{self.default_port}")
        
    def _init_variables(self):
        self._previous_button_lock = Lock()
        with self._previous_button_lock:
            self.previous_button = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.android_app_ip_lock = Lock()
        with self.android_app_ip_lock:
            self.android_app_ip = None

        self._send_lock = Lock()
        self._receive_lock = Lock()

    def _init_threads(self):
        self.gamepad_thread_lock = Lock()
        with self.gamepad_thread_lock:
            self._is_running = True
            self.gamepad_thread = threading.Thread(target = self._listen_udp, daemon=True)
            self.gamepad_thread.start()
            
    def _init_pub_sub_msg(self):
        self.datapub = self.create_publisher(BetterGamePad, '/pad', 10)
        self.default_value = BetterGamePad()
        self.default_value.left_analog_x = 128
        self.default_value.left_analog_y = 128 
        self.default_value.right_analog_x = 128
        self.default_value.right_analog_y = 128 

    def _init_sockets(self):
        self.temporary_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.temporary_udp.connect(('8.8.8.8', 80))  #Google's DNS
        self.ip = self.temporary_udp.getsockname()[0]
        self.temporary_udp.close()
        self.default_port = 55555
 
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x10)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        try:
            self.udp_socket.bind((self.ip, self.default_port))
            self.get_logger().info(f"Successfully bound to {self.ip}:{self.default_port}")
        except Exception as e:
            self.get_logger().error(f"[{self.default_port}]Bind failed: {e}")
            self.get_logger().info("Available IPs:")
            for interface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(interface)
            self.get_logger().info(f"{interface}: {addrs.get(netifaces.AF_INET)}")
        
    def _listen_gamepad(self, data):
            msg = BetterGamePad()
            with self._previous_button_lock:
                msg.previous_button_up = bool(self.previous_button[0])
                msg.previous_button_left = bool(self.previous_button[1])
                msg.previous_button_right = bool(self.previous_button[2])
                msg.previous_button_down = bool(self.previous_button[3])
                msg.previous_button_y = bool(self.previous_button[4])
                msg.previous_button_x = bool(self.previous_button[5])
                msg.previous_button_b = bool(self.previous_button[6])
                msg.previous_button_a = bool(self.previous_button[7])
                
                msg.previous_button_lt = bool(self.previous_button[8])
                msg.previous_button_lb = bool(self.previous_button[9])
                msg.previous_button_rt = bool(self.previous_button[10])
                msg.previous_button_rb = bool(self.previous_button[11])
                msg.previous_button_m1 = bool(self.previous_button[12])
                msg.previous_button_m2 = bool(self.previous_button[13])

                msg.left_analog_x = data[0] if len(data) > 0 else 128
                msg.left_analog_y = data[1] if len(data) > 1 else 128
                msg.right_analog_x = data[2] if len(data) > 2 else 128
                msg.right_analog_y = data[3] if len(data) > 3 else 128

                if len(data) >= 6:
                    msg.dpad_up = bool(data[4] & 1)
                    msg.dpad_left = bool(data[4] & 2)
                    msg.dpad_right = bool(data[4] & 4)
                    msg.dpad_down = bool(data[4] & 8)
                    
                    msg.button_y = bool(data[4] & 16)
                    msg.button_x = bool(data[4] & 32)
                    msg.button_b = bool(data[4] & 64)
                    msg.button_a = bool(data[4] & 128)

                    msg.button_lt = bool(data[5] & 1)
                    msg.button_lb = bool(data[5] & 2)
                    msg.button_rt = bool(data[5] & 4)
                    msg.button_rb = bool(data[5] & 8)
                    
                    msg.button_m1 = bool(data[5] & 16)
                    msg.button_m2 = bool(data[5] & 32)
                
                    self.previous_button = [msg.dpad_up, msg.dpad_left, msg.dpad_right, msg.dpad_down, 
                                            msg.button_y, msg.button_x, msg.button_b, msg.button_a, 
                                            msg.button_lt, msg.button_lb, msg.button_rt, msg.button_rb, 
                                            msg.button_m1, msg.button_m2]
                    self.datapub.publish(msg)
                else:
                    with self._previous_button_lock:
                        self.previous_button = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        self.datapub.publish(self.default_value)
                        
    def _listen_udp(self):
        while rclpy.ok() and self._is_running and self.udp_socket.fileno() != -1:
            try:
                data, addr = self.udp_socket.recvfrom(8)
                with self.android_app_ip_lock:
                    if self.android_app_ip is None:
                        self.android_app_ip = addr
                if data == b'TESTING!':  
                    self.get_logger().info(f"[{self.default_port}]Received test packet from {addr}")
                    continue
                else:
                    self._listen_gamepad(data)
            except Exception as e:
                self.get_logger().error(f"[ERROR] UDP received error: {e}")                   

    def _stop(self):
        with self.gamepad_thread_lock:
            if self._is_running:
                self._is_running = False

        try:
            if self.udp_socket:
                self.udp_socket.shutdown(socket.SHUT_RDWR)
                self.udp_socket.close()
        except:
            pass

        if self.gamepad_thread and self.gamepad_thread.is_alive():  
            try:
                self.gamepad_thread.join(timeout=0.5)
                self.gamepad_thread = None
            except Exception as e:
                self.get_logger().warn("Failed to join gamepad thread, Please restart the node!")
                self.gamepad_thread = None
        self.get_logger().info(f"Clean up successfully!")

def main(args=None):
    rclpy.init(args=args)
    default_node = gamepad_default_legacy_node()
    try:
        rclpy.spin(default_node)
    except KeyboardInterrupt:
        pass
    finally:
        if default_node._is_running:
           default_node._stop()
        default_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
