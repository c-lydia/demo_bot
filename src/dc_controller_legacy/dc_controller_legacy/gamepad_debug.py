import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from custom_messages.msg import DebugMsg, PisethDebug, KsattraDebug, ThreeEncoders
import socket
import threading    
import subprocess   
import importlib    
import json         
import time
from threading import Lock  
import netifaces


class gamepad_debug_legacy_node(Node):
    def __init__(self):
        super().__init__("gamepad_debug_legacy")
        self._init_variables()
        self._init_sockets()
        self._init_threads()

        self.get_logger().info("PLEASE MAKE SURE \"gamepad_default_legacy\" IS RUNNING")
        self.get_logger().info("[RUNNING] Waiting for commands...")

    def _init_variables(self):
        self.special_topics = ["/pad", "/debug1", "/debug2", "/three_encoders", "/piseth_debug", "/ksattra_debug"]
        self.pause_lock = Lock()
        with self.pause_lock:
            self.paused = False
        self.sub_lock = Lock()
        with self.sub_lock:
            self.current_sub = None
            self._cached_topics = []
            self._last_topic_refresh = 0.0
            self._is_subscribed = False

        self.latest_data_lock = Lock()
        with self.latest_data_lock:
            self.latest_data = None
            self.latest_topic = None
        
        self.android_app_ip_lock = Lock()
        with self.android_app_ip_lock:
            self.android_app_ip = None

    def _init_sockets(self):
        self.default_port = 55555
        self.debug_port = 55556
        self.display_port = 55557
        self.temporary_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.temporary_socket.connect(("8.8.8.8", 80))
        self.debug_ip = self.temporary_socket.getsockname()[0]
        self.display_ip = self.temporary_socket.getsockname()[0]
        self.temporary_socket.close()

        self.debug_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.debug_socket.bind((self.debug_ip, self.debug_port))
            self.debug_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
            self.debug_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
            self.get_logger().info(f"Successfully bound to {self.debug_ip}:{self.debug_port}")
        except Exception as e:
            self.get_logger().error(f"Bind failed: {str(e)}")
            self.get_logger().info("Available IPs:")
            for interface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(interface)
                self.get_logger().info(f"{interface}: {addrs.get(netifaces.AF_INET)}")
      
        self.display_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        try:
            self.display_socket.bind((self.display_ip, self.display_port))
            self.display_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.display_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048) 
            self.get_logger().info(f"Successfully bound to {self.display_ip}:{self.display_port}")
        except Exception as e:
            self.get_logger().error(f"Bind failed: {str(e)}")
            self.get_logger().info("Available IPs:")
            for interface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(interface)
                self.get_logger().info(f"{interface}: {addrs.get(netifaces.AF_INET)}")

    def _init_threads(self):
        self.udp_thread_lock = Lock()
        with self.udp_thread_lock:
            self.listen_command_thread = threading.Thread(target=self._listen_udp, daemon=True)
            self.listen_display_port_thread = threading.Thread(target=self._listen_display_port, daemon = True)
            self._is_running = True
            self.listen_command_thread.start()
            self.listen_display_port_thread.start()

    def _listen_udp(self):
        while rclpy.ok() and self._is_running:
            try:
                dataDebug,addrDebug = self.debug_socket.recvfrom(1024)
                if self.android_app_ip is None:
                    with self.android_app_ip_lock:  
                        self.android_app_ip = addrDebug    
                if dataDebug == b'TESTING!':
                    self.get_logger().info(f"[ANDROID] recieved test package from: {addrDebug}")
                    continue
                else:
                    self._listen_debug_port(dataDebug, addrDebug)
                    
            except Exception as e:
                self.get_logger().error(f"[ERROR] UDP received error: {str(e)}")
            except socket.timeout:
                continue
    
    def _listen_debug_port(self, data, addr):
        data_string = data.decode('utf-8')
        if not data_string:
            self.get_logger().info("Received empty message")
            return
        elif data_string.startswith("[ERROR]"):
            self.get_logger().error(f"[ANDROID] {data_string}")
            return
        elif data_string.startswith("[DEBUG]"):
            self.get_logger().info(f"[ANDROID] {data_string}")
            return
    
        else:
            try: 
                cmd = json.loads(data.decode())
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON: {data}")
                return
            if cmd.get('command') == 'list':
                self.get_logger().info("[DEBUG] Listing all active topics...")
                try:
                    active_topics = self._list_all_topic()
                    response = json.dumps({"topics": active_topics})
                    self.debug_socket.sendto(response.encode(), addr)
                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f"[ERROR] CLI error: {str(e)}")
                    
            elif cmd.get('command') == 'subscribe':
                self.get_logger().info("[DEBUG] Subscribing to a topic...")
                topic = cmd['topic']
                self._subscribe_topic(topic)
            elif cmd.get('command') == 'unsubscribe':
                with self.sub_lock:
                    if self.current_sub:
                        self.destroy_subscription(self.current_sub)
                        self.current_sub = None
                        self._is_subscribed = False
                self.get_logger().info("[DEBUG] Unsubscribed from topic")

    def _listen_display_port(self):
        while rclpy.ok() and self._is_running:
            try:
                dataDisplay,addrDisplay = self.display_socket.recvfrom(64)
                if dataDisplay == b'TESTING!':
                    self.get_logger().info(f"[ANDROID] recieved test package from: {addrDisplay}")
                    continue
            except Exception as e:
                self.get_logger().error(f"[ERROR] UDP received error: {str(e)}")
            except socket.timeout:
                continue
    
    def _format_message(self, topic, data_dict):
        if topic in ["/debug1", '/debug2']:
            parts = [""]
            counter = 0
            for k, v in data_dict.items():
                if k.startswith('SLOT_TYPES'):
                    continue
                counter += 1
                if isinstance(v, (int)):
                    parts.append(f"{k}={v}   ") 
                elif isinstance(v, float):
                    if k == "speed":
                        parts.append(f"{k}={v:.1f}   ") 
                    elif k in ["rf_front", "rf_sb", "rf_sf"]:
                        parts.append(f"{k}={v:.3f}   ")
                    else:
                        parts.append(f"{k}={v:.2f}   ")
                elif isinstance(v, str):
                    parts.append(f"{k}='{v}'   ") 
               
                
                if counter in [3, 6, 9, 10, 11, 12]:
                    parts.append(f"\n\n")

            return "".join(parts) + "\n"
        elif topic == "/ksattra_debug":
            parts = []
            counter = 0
            expected_keys = ['int_a', 'int_b', 'int_c', 'float_a', 'float_b', 'float_c',
                            'bool_a', 'bool_b', 'bool_c','log_a', 'log_b', 'log_c']
            for k, v in data_dict.items():
                if k not in expected_keys:  
                    continue
                counter += 1
                if isinstance(v, bool):     
                    parts.append(f"{k}={v}   ")
                elif isinstance(v, int):
                    parts.append(f"{k}={v}   ")
                elif isinstance(v, float):
                    parts.append(f"{k}={v:.3f}   ")
                elif isinstance(v, str):
                    parts.append(f"{k}='{v}'   ")
                else:
                    continue

                if counter in [3, 6, 9, 10, 11, 12]:
                    parts.append("\n\n")

            return "".join(parts) + "\n"
        elif topic == "/piseth_debug":
            parts = []
            counter = 0
            
            for k, v in data_dict.items():
                counter += 1
                if isinstance(v, str) and v != "":
                    parts.append(f"{k} = {v}\n")
                
            return "".join(parts) + "\n"
                
        else:
            parts = [f"[{topic}]"]
            for k, v in data_dict.items():
                if isinstance(v, (bool)):
                     parts.append(f"{k}={v}   ") 
                elif isinstance(v, (int)):
                    parts.append(f"{k}={v}   ") 
                elif isinstance(v, float):
                    parts.append(f"{k}={v:.3f}   ") 
                elif isinstance(v, str):
                    parts.append(f"{k}='{v}'   ") 
                
            return " ".join(parts) + "\n"
    
    def _send_msg(self, formatted_msg):   
        if not self._is_subscribed:
            return
        with self.android_app_ip_lock:
            if self.android_app_ip is None:
                return
        try:    
            self.display_socket.sendto(formatted_msg.encode('utf-8'), (self.android_app_ip[0], self.display_port))
            return
                
        except Exception as e:
            self.get_logger().error(f"[ERROR] send failed: {str(e)}")
            return

    def _msg_preparation(self, msg, topic_name):
        with self.pause_lock:
            if self.paused:
                return
        try:
            if (topic_name in self.special_topics):
                msg_dict  = self._convert_msg_special(msg, topic_name) 
            else:
                msg_dict = self._convert_msg(msg)
            formatted_msg = self._format_message(topic_name, msg_dict)
            self._send_msg(formatted_msg)

        except Exception as e:
            self.get_logger().error(f"[ERROR] Message error: {str(e)}")
    
    def _convert_msg_special(self, msg, topic_name):
        if topic_name == "/odom":
            base = {'[time]' : int(time.time())}
            base.update({
                'linear.x' : msg.twist.twist.linear.x,
                'linear.y' : msg.twist.twist.linear.y,
                'angular.z' : msg.twist.twist.angular.z
            })
            return base
        elif topic_name in ["/debug1", "/debug2"]:
            base = {}
            base.update({
                'correct_x' : msg.correct_x,
                'correct_y' : msg.correct_y,
                'correct_yaw' : msg.correct_yaw,
                'x' : msg.x_value,
                'y' : msg.y_value,
                'yaw' :msg.yaw_value,
                'error_x' : msg.error_x,
                'error_y' : msg.error_y,
                'error_yaw' : msg.error_yaw,
                'distance' : msg.distance,
                'speed' : msg.speed,
                'rf_front' : msg.rf_front,
                'rf_sf' : msg.rf_sf,
                'rf_sb' : msg.rf_sb,
            })
            return base
        elif topic_name == "/pad":
            base = {'[time]' : int(time.time())}
            base.update({
                'left_x' : msg.left_analog_x,
                'left_y' : msg.left_analog_y,
                'right_x' : msg.right_analog_x,
                'right_y' : msg.right_analog_y,
            })
            return base
        elif topic_name == "/three_encoders":
            base = {'[time]' : int(time.time())}
            base.update({
                'encoder_a' : msg.encoder_a,
                'encoder_b' : msg.encoder_b,
                'encoder_c' : msg.encoder_c,
            })
            return base
        elif topic_name == "/ksattra_debug":
            base = {}
            base.update({
                'int_a' : msg.int_a,
                'int_b' : msg.int_b,
                'int_c' : msg.int_c,

                'float_a' : msg.float_a,
                'float_b' : msg.float_b,
                'float_c' : msg.float_c,

                'bool_a' : msg.bool_a,
                'bool_b' : msg.bool_b,
                'bool_c' : msg.bool_c,

                'log_a' : msg.log_a,
                'log_b' : msg.log_b,
                'log_c' : msg.log_c,
            })
            return base
        elif topic_name == "/piseth_debug":
            base = {}
            base.update({
                'log_a' : msg.log_a,
                'log_b' : msg.log_b,
                'log_c' : msg.log_c,
                'log_d' : msg.log_d,
                'log_e' : msg.log_e,
            })
            return base
    def _convert_msg(self, msg):
        base = {'[time]' : int(time.time())}
        msg_dict = {}

        for field in msg.__slots__:
            value = getattr(msg, field)

            if hasattr(value, '__slots__'):
                msg_dict[field] = self._convert_msg(value)
            elif isinstance(value, (list, tuple)):
                msg_dict[field] = [
                    self._convert_msg(x) if hasattr(x, '__slots__') else x
                    for x in value
                ]
            else:
                msg_dict[field] = value

        base.update(self._flatten_msg(msg_dict))
        return base
        
    def _flatten_msg(self, data, prefix=''):
        items = {}
        for k, v in data.items():
            new_key = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                items.update(self._flatten_msg(v, new_key))
            elif isinstance(v, (list, tuple)):
                for i, item in enumerate(v):
                    items.update(self._flatten_msg({str(i): item}, new_key))
            else:
                items[new_key] = v
        return items

    def _subscribe_topic(self, topic_name):  
        with self.sub_lock:
            try:
                if self.current_sub:
                    self.destroy_subscription(self.current_sub)
                    self.current_sub = None
                    self._is_subscribed = False
                
                with self.latest_data_lock:
                    self.latest_data = None
                    self.latest_topic = None

                for name, types in self.get_topic_names_and_types():
                    if name == topic_name:
                        topic_type = types[0]
                        break
                if not topic_type:
                    self.get_logger().error(f"[ERROR] Topic {topic_name} not found!")
                    return
                    
                parts = topic_type.split('/')
                pkg = parts[0]       
                msg = parts[-1]      
            
                module = __import__(f"{pkg}.msg", fromlist=[msg])
                msg_class = getattr(module, msg)
                
                self.current_sub = self.create_subscription(msg_class, topic_name,lambda msg: self._msg_preparation(msg, topic_name), 25)
                
                self.get_logger().info(f"[DEBUG] Successfully subscribed to {topic_name}")
                self._is_subscribed = True

            except Exception as e:
                self.get_logger().error(f"[ERROR] Failed to subscribe: {str(e)}")

    def _list_all_topic(self):
        if time.time() - self._last_topic_refresh > 5.0:
            result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, check=True)
            self._cached_topics = result.stdout.splitlines()
            self._last_topic_refresh = time.time()
        return self._cached_topics       
    
    def _stop(self):
        with self.udp_thread_lock:
            if self._is_running:
                self._is_running = False
        
        with self.sub_lock:
            if self.current_sub:
                self.destroy_subscription(self.current_sub)
                self.current_sub = None
                self._is_subscribed = False

        with self.latest_data_lock:
            self.latest_data = None
            self.latest_topic = None

        try:
            self.debug_socket.shutdown(socket.SHUT_RDWR)
            self.debug_socket.close()
        except:
            pass
        
        try:
            self.display_socket.shutdown(socket.SHUT_RDWR)
            self.display_socket.close()
        except:
            pass

            
        if self.listen_command_thread and self.listen_command_thread.is_alive(): 
            try:
                self.listen_command_thread.join(timeout=1.0)
                self.listen_command_thread = None
            except Exception as e:
                self.get_logger().warn("Failed to join debug thread, Please restart the node!")
                self.listen_command_thread = None
        if self.listen_display_port_thread and self.listen_display_port_thread.is_alive():
            try:
                self.listen_display_port_thread.join(timeout=1.0)
                self.listen_display_port_thread = None
            except Exception as e:
                self.get_logger().warn("Failed to join display thread, Please restart the node!")
                self.listen_display_port_thread = None
        
        self.debug_socket = None
        self.display_socket =None

        
        self.get_logger().info(f"Clean up successfully!")

def main(args=None):
    rclpy.init(args=args)
    debug_node = gamepad_debug_legacy_node()
    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        pass
    finally:
        debug_node._stop()
        debug_node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
