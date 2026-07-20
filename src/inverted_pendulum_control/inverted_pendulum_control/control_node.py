import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from scipy.linalg import solve_continuous_are


class InvertedPendulumController(Node):

    def __init__(self):
        super().__init__('inverted_pendulum_controller')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64,
            '/cart_force_cmd',
            10
        )

        # Sistem parametreleri
        M = 3.0
        m = 1.0
        L = 0.5
        r = 0.01
        g = 9.81

        # Pivot ile kütle merkezi arasındaki mesafe
        l = L / 2.0

        # Sarkacın kütle merkezi etrafındaki ataleti
        I = (m / 12.0) * (3.0 * r**2 + L**2)

        # Pivot etrafındaki toplam atalet
        J = I + m * l**2

        # Hareket denklemlerini matris formunda çözerken ortaya çıkan ortak payda (determinant)
        delta = (M + m) * J - (m * l)**2

        # State-space matrisi
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -(m**2 * g * l**2) / delta, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, m * g * l * (M + m) / delta, 0.0]
        ])

        B = np.array([
            [0.0],
            [J / delta],
            [0.0],
            [-m * l / delta]
        ])

        # LQR ağırlıkları
        Q = np.diag([
            10.0,
            1.0,
            100.0,
            1.0
        ])

        R = np.array([
            [0.1]
        ])

        # Riccati denklemi
        P = solve_continuous_are(A, B, Q, R)

        # LQR kazancı
        self.K = np.linalg.inv(R) @ B.T @ P

        self.get_logger().info(f'K matrix: {self.K}')

    def joint_state_callback(self, msg):

        cart_index = msg.name.index('cart_rail_joint')
        pendulum_index = msg.name.index('pendulum_cart_joint')

        # State değerleri
        x = msg.position[cart_index]
        x_dot = msg.velocity[cart_index]

        theta = msg.position[pendulum_index]
        theta_dot = msg.velocity[pendulum_index]

        # State vektörü
        state = np.array([
            [x],
            [x_dot],
            [theta],
            [theta_dot]
        ])

        # LQR kontrol kuvveti
        force = -self.K @ state

        # ROS mesajına dönüştür
        force_msg = Float64()
        force_msg.data = float(force[0, 0])

        # Kuvveti publish et
        self.publisher.publish(force_msg)


def main(args=None):
    rclpy.init(args=args)

    node = InvertedPendulumController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()