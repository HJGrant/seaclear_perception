import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import psutil
import time

class JetsonNetworkMonitor(Node):
    def __init__(self):
        super().__init__('jetson_network_monitor')

        # 1. Configuration
        self.declare_parameter('interface', 'eno1')
        self.interface = self.get_parameter('interface').value
        
        # 2. Standard Publishers (for plotting)
        self.pub_in = self.create_publisher(Float32, '~/network/incoming_mbps', 10)
        self.pub_out = self.create_publisher(Float32, '~/network/outgoing_mbps', 10)

        # 3. Setup Diagnostic Updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("jetson_orin_nx")
        # This creates an entry in the Diagnostic Summary called "Network Bandwidth"
        self.updater.add("Network Bandwidth", self.produce_diagnostics)

        # 4. Tracking Variables
        try:
            stats = psutil.net_io_counters(pernic=True)[self.interface]
            self.last_recv = stats.bytes_recv
            self.last_sent = stats.bytes_sent
            self.last_time = time.time()
        except KeyError:
            self.get_logger().error(f"Interface {self.interface} not found at startup!")
            exit(1)

        # Current values for the diagnostic updater to read
        self.mbps_in = 0.0
        self.mbps_out = 0.0

        # 5. High-speed timer for calculation (1Hz)
        self.create_timer(1.0, self.update_stats)

    def update_stats(self):
        """Calculates the bandwidth and publishes raw topics."""
        try:
            stats = psutil.net_io_counters(pernic=True)[self.interface]
            current_time = time.time()
            dt = current_time - self.last_time

            if dt > 0:
                # Calculate Mbps
                self.mbps_in = ((stats.bytes_recv - self.last_recv) * 8) / (1024**2) / dt
                self.mbps_out = ((stats.bytes_sent - self.last_sent) * 8) / (1024**2) / dt

                # Publish Float32 for Plotting
                self.pub_in.publish(Float32(data=self.mbps_in))
                self.pub_out.publish(Float32(data=self.mbps_out))

                # Store for next loop
                self.last_recv = stats.bytes_recv
                self.last_sent = stats.bytes_sent
                self.last_time = current_time
            
            # Trigger the updater to publish to /diagnostics
            self.updater.update()

        except KeyError:
            pass # Interface might have flickered off

    def produce_diagnostics(self, stat):
        """Callback for the Diagnostic Updater to build the status message."""
        
        # Set the Summary Level based on thresholds
        if self.mbps_out > 800.0 or self.mbps_in > 800.0:
            stat.summary(DiagnosticStatus.WARN, "High Load: Approaching Link Capacity")
        elif self.mbps_out < 0.01 and self.mbps_in < 0.01:
            stat.summary(DiagnosticStatus.OK, "Idle: No Traffic Detected")
        else:
            stat.summary(DiagnosticStatus.OK, "Nominal")

        # Add data rows to the 'Diagnostic Details' panel
        stat.add("Interface", self.interface)
        stat.add("Incoming (Mbps)", f"{self.mbps_in:.2f}")
        stat.add("Outgoing (Mbps)", f"{self.mbps_out:.2f}")
        
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = JetsonNetworkMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()