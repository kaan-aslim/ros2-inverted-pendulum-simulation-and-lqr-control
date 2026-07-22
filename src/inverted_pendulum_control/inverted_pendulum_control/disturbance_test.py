import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DisturbanceTest(Node):

    def __init__(self) -> None:
        super().__init__("disturbance_test")

        self.declare_parameter("torque", 0.5)
        self.declare_parameter("duration", 0.05)

        self.torque = float(self.get_parameter("torque").value)
        self.duration = float(self.get_parameter("duration").value)

        self.publisher = self.create_publisher(
            Float64,
            "/pendulum_force_cmd",
            10,
        )

        self.started = False
        self.stop_timer = None

        # Bridge bağlantısının kurulması için kısa bekleme
        self.start_timer = self.create_timer(0.5, self.start_disturbance)

    def start_disturbance(self) -> None:
        if self.started:
            return

        self.started = True
        self.start_timer.cancel()

        msg = Float64()
        msg.data = self.torque
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Applying {self.torque:.3f} N·m for "
            f"{self.duration:.3f} s"
        )

        self.stop_timer = self.create_timer(
            self.duration,
            self.stop_disturbance,
        )

    def stop_disturbance(self) -> None:
        if self.stop_timer is not None:
            self.stop_timer.cancel()

        msg = Float64()
        msg.data = 0.0

        # Sıfırlama mesajının alınma ihtimalini artırmak için birkaç kez gönder
        for _ in range(5):
            self.publisher.publish(msg)

        self.get_logger().info("Disturbance removed.")
        self.create_timer(0.2, self.shutdown)

    def shutdown(self) -> None:
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisturbanceTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()