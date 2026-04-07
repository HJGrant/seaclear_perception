import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class VelocityPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'velocity_commander'
        super().__init__('velocity_commander')
        
        # Create the publisher
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            '/mavros/setpoint_velocity/cmd_vel', 
            10
        )
        
        # Create a timer that runs at 20Hz (0.05 seconds)
        self.timer_period = 0.05  
        self.timer = self.create_timer(self.timer_period, self.publish_velocity)
        
        # State management
        self.start_time = self.get_clock().now()
        self.duration = 5.0  # Move forward for 5 seconds
        self.is_stopping = False

    def publish_velocity(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed_time < self.duration:
            # Move forward at 2 m/s
            msg.twist.linear.x = 2.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            
            self.get_logger().info('Moving Forward...', throttle_duration_sec=1.0)
        elif elapsed_time < (self.duration + 2.0):
            # Stop command for 2 seconds
            msg.twist.linear.x = 0.0
            self.get_logger().warn('Stopping...', throttle_duration_sec=1.0)
        else:
            # Shutdown the node after the routine is done
            self.get_logger().info('Mission Complete. Shutting down.')
            raise SystemExit 

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()