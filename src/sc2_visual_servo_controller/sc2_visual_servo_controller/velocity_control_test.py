import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Crucial: enables do_transform_point

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        
        # 1. Declare Parameters
        self.declare_parameter('velocity', 0.3)
        self.declare_parameter('axis', 'x')      
        self.declare_parameter('duration', 5.0)  
        
        # 2. Setup TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            '/mavros/setpoint_velocity/cmd_vel', 
            10
        )
        
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.publish_velocity)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("Velocity Controller Node Initialized")

    def publish_velocity(self):
        # Calculate elapsed time
        curr_time = self.get_clock().now()
        elapsed_time = (curr_time - self.start_time).nanoseconds / 1e9
        
        # 3. Create a PointStamped for the body-frame velocity
        # We use PointStamped because do_transform_point is the most reliable 
        # way to rotate a 3D vector using the tf2_geometry_msgs library.
        v_body = PointStamped()
        v_body.header.frame_id = 'base_link'
        v_body.header.stamp = curr_time.to_msg()

        duration = self.get_parameter('duration').value
        
        if elapsed_time < duration:
            axis = self.get_parameter('axis').value.lower()
            vel = self.get_parameter('velocity').value
            
            if axis == 'x': v_body.point.x = vel
            elif axis == 'y': v_body.point.y = vel
            elif axis == 'z': v_body.point.z = vel
        elif elapsed_time >= (duration + 2.0):
            self.get_logger().info('Mission Complete. Shutting down.')
            raise SystemExit

        # 4. Lookup the transform and transform the velocity
        try:
            # Look up transform from map to base_link
            # Using rclpy.time.Time() gets the latest available transform
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time()
            )
            
            # Transform the body-frame point into the map-frame
            v_map = tf2_geometry_msgs.do_transform_point(v_body, trans)

            # 5. Prepare and publish the TwistStamped message
            msg = TwistStamped()
            msg.header.stamp = curr_time.to_msg()
            msg.header.frame_id = "map"
            
            # Map the transformed points back to linear velocities
            msg.twist.linear.x = v_map.point.x
            msg.twist.linear.y = v_map.point.y
            msg.twist.linear.z = v_map.point.z
            
            self.vel_pub.publish(msg)
            self.get_logger().info(
                f'Moving {self.get_parameter("axis").value}: '
                f'MapVel X={msg.twist.linear.x:.2f}, Y={msg.twist.linear.y:.2f}', 
                throttle_duration_sec=1.0
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_link to map (is your static_tf running?): {ex}', 
                throttle_duration_sec=2.0
            )

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