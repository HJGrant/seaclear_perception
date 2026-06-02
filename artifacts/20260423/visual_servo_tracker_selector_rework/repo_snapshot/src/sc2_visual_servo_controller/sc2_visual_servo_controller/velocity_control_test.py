import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs # Required for do_transform_vector3

class Sc2VelocityPublisher(Node):
    def __init__(self):
        super().__init__('sc2_velocity_commander')
        
        # 1. Parameters
        self.declare_parameter('velocity', 0.3)
        self.declare_parameter('axis', 'x')      
        self.declare_parameter('duration', 5.0)  
        
        # 2. TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            '/mavros/setpoint_velocity/cmd_vel', 
            10
        )
        
        self.timer = self.create_timer(0.05, self.publish_velocity)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("SC2 to BaseLink Velocity Node Initialized")

    def publish_velocity(self):
        curr_time = self.get_clock().now()
        elapsed_time = (curr_time - self.start_time).nanoseconds / 1e9
        duration = self.get_parameter('duration').value

        # 3. Define velocity in the SC2 frame
        v_sc2 = Vector3Stamped()
        v_sc2.header.frame_id = 'sc2_base_link'
        v_sc2.header.stamp = curr_time.to_msg()

        if elapsed_time < duration:
            axis = self.get_parameter('axis').value.lower()
            vel = self.get_parameter('velocity').value
            if axis == 'x': v_sc2.vector.x = vel
            elif axis == 'y': v_sc2.vector.y = vel
            elif axis == 'z': v_sc2.vector.z = vel
        elif elapsed_time >= (duration + 1.0):
            raise SystemExit

        # 4. Transform SC2 velocity to BaseLink velocity
        try:
            # We look up the transform that describes sc2_base_link relative to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'sc2_base_link', 
                rclpy.time.Time()
            )
            
            # Use Vector3 transformation so we only rotate the direction
            v_base = tf2_geometry_msgs.do_transform_vector3(v_sc2, transform)

            # 5. Publish in base_link frame
            msg = TwistStamped()
            msg.header.stamp = curr_time.to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = v_base.vector.x
            msg.twist.linear.y = v_base.vector.y
            msg.twist.linear.z = v_base.vector.z
            
            self.vel_pub.publish(msg)

        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF: {ex}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = Sc2VelocityPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()