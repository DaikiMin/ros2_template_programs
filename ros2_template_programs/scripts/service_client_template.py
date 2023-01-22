from ros2_custom_msg.msg import Expression
from ros2_custom_msg.srv import Calculation
import rclpy
from rclpy.node import Node

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client_template')
        self.cli = self.create_client(Calculation, 'calculate_two_numbers')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter( 'a', 1)
        self.declare_parameter( 'b', 1)
        self.count = 0

    def timer_callback(self):
        req = Calculation.Request()
        req.a = self.get_parameter("a").get_parameter_value()
        req.b = self.get_parameter("b").get_parameter_value()
        if self.count%4 == 0:
            req.calculate = '+'
        elif self.count%4 == 1:
            req.calculate = '-'
        elif self.count%4 == 2:
            req.calculate = '*'
        else:
            req.calculate = '/'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        res = self.future.result()
        self.get_logger().info('Result: %.2f\n', res.result)
        self.count += 1
        return

def main():
    rclpy.init()

    client = ServiceClient()
    try:
        response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        client.get_logger().info( 'Result of add_two_ints: for %d + %d = %d' %(int(sys.argv[1]), int(sys.argv[2]), response.sum))
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()