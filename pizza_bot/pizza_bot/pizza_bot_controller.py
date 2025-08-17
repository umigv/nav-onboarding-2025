import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from pizza_bot_interfaces.msg import Order, Coord
from pizza_bot_interfaces.srv import NavigateToCoord, DeliverPizza
from pizza_bot_interfaces.action import MakePizza


class PizzaBotController(Node):

    def __init__(self):
        super().__init__('pizza_bot_controller_node')
        # Subscribers
        self._order_subscriber = self.create_subscription(
            Order,
            'orders',
            self.order_callback,
            10
        )

        # Publishers
        self._received_order_publisher = self.create_publisher(Order, 'received_orders', 10)

        # Clients
        self._navigate_client = self.create_client(NavigateToCoord, 'navigate_to_coord')
        self._make_pizza_client = ActionClient(self, MakePizza, 'make_pizza')
        self._deliver_pizza_client = self.create_client(DeliverPizza, 'deliver_pizza')

    # ------------------- Order Handling ------------------- #
    def order_callback(self, order: Order):
        self.get_logger().info("---------- Publishing received order ----------")
        self._received_order_publisher.publish(order)

        self.call_navigate_service(
            order.pizza_place_coord,
            lambda future: self.pizza_place_navigation_callback(future, order)
        )

    # ------------------- Navigation ------------------- #
    def call_navigate_service(self, goal: Coord, response_callback):
        while not self._navigate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("navigate_to_coord service not available, waiting again...")

        goal_request = NavigateToCoord.Request()
        goal_request.goal = goal

        future = self._navigate_client.call_async(goal_request)
        future.add_done_callback(response_callback)

    def pizza_place_navigation_callback(self, future, order: Order):
        response = future.result()
        navigation_succeeded = response.success
        pizza_place_coord = order.pizza_place_coord
        pizza_place_name = order.pizza_place

        if not navigation_succeeded:
            self.get_logger().info(
                f"Failed to navigate to {pizza_place_name} at ({pizza_place_coord.x}, {pizza_place_coord.y})"
            )
            return

        self.get_logger().info(
            f"Successfully navigated to {pizza_place_name} at ({pizza_place_coord.x}, {pizza_place_coord.y})"
        )
        self.send_pizza_order(order)

    # ------------------- Action: MakePizza ------------------- #
    def send_pizza_order(self, order: Order):
        while not self._make_pizza_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("make_pizza action not available, waiting again...")

        goal_msg = MakePizza.Goal()
        goal_msg.pizza_type = order.pizza_type

        self._make_pizza_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(
            lambda future: self.goal_response_callback(future, order)
        )

    def goal_response_callback(self, future, order: Order):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Order was rejected by restaurant")
            return

        self.get_logger().info("Order accepted by restaurant, waiting for pizza")
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.result_callback(future, order)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback from restaurant: {feedback.pizza_status}")

    def result_callback(self, future, order: Order):
        result = future.result().result
        status = future.result().status

        if status == 2:  # ABORTED
            self.get_logger().error("Order was aborted")
            return
        elif status == 3:  # CANCELED
            self.get_logger().error("Order was canceled")
            return

        if not result.pizza_finished:
            self.get_logger().info(
                f"{order.pizza_place} failed to complete the order for a {order.pizza_type} pizza"
            )
            return

        self.get_logger().info(
            f"{order.pizza_place} completed the order for a {order.pizza_type} pizza"
        )

        self.call_navigate_service(
            order.customer_coord,
            lambda future: self.customer_navigation_callback(future, order)
        )

    # ------------------- Delivery ------------------- #
    def customer_navigation_callback(self, future, order: Order):
        response = future.result()
        navigation_succeeded = response.success
        customer_coord = order.customer_coord

        if not navigation_succeeded:
            self.get_logger().info(
                f"Failed to navigate to customer's house at ({customer_coord.x}, {customer_coord.y})"
            )
            return

        self.get_logger().info(
            f"Successfully navigated to customer's house at ({customer_coord.x}, {customer_coord.y})"
        )
        self.deliver_pizza(order.pizza_type)

    def deliver_pizza(self, pizza_type: str):
        while not self._deliver_pizza_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("deliver_pizza service not available, waiting again...")

        request = DeliverPizza.Request()
        request.pizza_type = pizza_type

        future = self._deliver_pizza_client.call_async(request)
        future.add_done_callback(self.pizza_delivery_callback)

    def pizza_delivery_callback(self, future):
        response = future.result()
        if not response.success:
            self.get_logger().info("Failed to complete delivery")
            return
        self.get_logger().info("Successfully completed delivery")

def main(args=None):
    rclpy.init(args=args)
    node = PizzaBotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()