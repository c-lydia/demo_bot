import rclpy 
from rclpy.node import Node
from custom_messages.msg import ThreeEncoders, EncoderFeedback


class three_encoders_publisher(Node):
    def __init__(self):
        super().__init__('three_encoders_publisher_node')
        self.three_encoders_publisher = self.create_publisher(ThreeEncoders, '/three_encoders', 10)
        self.encoder_feedback_sub = self.create_subscription(EncoderFeedback, '/encoder_feedback', self.callback_encoder_feedback_sub, 10)
        self.publishing_timer = self.create_timer(0.01, self.publish_three_encoders_msg)
        self.encoder_a = 0.0
        self.encoder_b = 0.0
        self.encoder_c = 0.0

        self.desired_encoder_ids = [1, 2, 3]
        
        self.get_logger().info("three_encoders_publisher_node started successfully!")
    def callback_encoder_feedback_sub(self, msg : EncoderFeedback):
        if msg.can_id not in self.desired_encoder_ids:
            return
        if msg.can_id == self.desired_encoder_ids[0]:
            self.encoder_a = msg.speed
        elif msg.can_id == self.desired_encoder_ids[1]:
            self.encoder_b = msg.speed
        elif msg.can_id == self.desired_encoder_ids[2]:
            self.encoder_c = msg.speed
    
    def publish_three_encoders_msg(self):
        msg = ThreeEncoders()
        msg.encoder_a = self.encoder_a
        msg.encoder_b = self.encoder_b
        msg.encoder_c = self.encoder_c

        self.three_encoders_publisher.publish(msg)


def main(args = None):
    rclpy.init(args = args)
    node = three_encoders_publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


