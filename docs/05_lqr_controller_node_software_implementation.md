# LQR Controller Node Software Implementation

## Introduction

The previous documents developed the rigid-body model of the inverted pendulum, derived its nonlinear equations of motion, linearized the equations about the unstable upright equilibrium, and designed a Linear Quadratic Regulator (LQR).

This document explains how that mathematical model is implemented in `control_node.py` as a ROS 2 node. The implementation operates in two distinct phases:

1. During node initialization, it defines the rigid-body parameters, constructs the state-space matrices, solves the continuous-time algebraic Riccati equation (CARE), and calculates the LQR gain matrix.
2. Whenever a `JointState` message is received, it constructs the measured state vector, calculates the control force, and publishes that force to the cart.

The implemented feedback sequence is:

1. Receive the current cart and pendulum joint states.
2. Construct the state vector.
3. Calculate the LQR control force.
4. Publish the force command to the cart.
5. Repeat when the next `JointState` message arrives.

---

## 1. Controller Architecture

The controller implements the state-feedback law

$$
u=-K\mathbf{x}
$$

for the state vector

$$
\mathbf{x}=\begin{bmatrix}x\\
\dot{x}\\
\theta\\
\dot{\theta}\end{bmatrix}
$$

Here, $u=F$ is the scalar horizontal force applied to the cart. The controller receives the four measured states through `/joint_states` and publishes the calculated scalar force through `/cart_force_cmd`.

The overall controller architecture is illustrated below.

<p align="center">
    <img src="images/lqr_controller_architecture.png" alt="LQR Controller Architecture" width="900">
</p>

The node does not use a separate timer. Its control calculation is event-driven: `joint_state_callback()` runs once for every received `JointState` message. Consequently, the effective controller update rate is determined by the publication rate of `/joint_states` and the ROS 2 communication path.

---

## 2. Importing the Required Libraries

The controller begins with the following imports:

```python
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from scipy.linalg import solve_continuous_are
```

Each library has a specific role:

| Import | Purpose |
|---|---|
| `numpy` | Constructs vectors and matrices and performs matrix operations |
| `rclpy` | Provides the ROS 2 Python client library |
| `Node` | Provides the base class for the controller node |
| `JointState` | Carries the measured joint names, positions, and velocities |
| `Float64` | Carries the scalar force command |
| `solve_continuous_are` | Solves the continuous-time algebraic Riccati equation |

NumPy represents the matrices $A$, $B$, $Q$, $R$, $P$, and $K$, as well as the state vector $\mathbf{x}$. SciPy calculates the CARE solution $P$, while ROS 2 provides communication between the controller and the simulation.

---

## 3. Creating the ROS 2 Controller Node

The controller is implemented as a class derived from `rclpy.node.Node`:

```python
class InvertedPendulumController(Node):
```

Its constructor initializes the ROS 2 node:

```python
def __init__(self):
    super().__init__('inverted_pendulum_controller')
```

The node is registered with the name:

```text
inverted_pendulum_controller
```

The constructor performs every calculation that remains constant while the node is running:

- creates the subscriber and publisher,
- defines the physical parameters,
- calculates the rigid pendulum's inertial properties,
- constructs the state-space matrices,
- defines the LQR weighting matrices,
- solves the CARE,
- calculates and stores the LQR gain matrix.

The gain is stored as `self.K` because it must remain available to the callback function throughout the node's lifetime.

---

## 4. ROS 2 Communication Interfaces

### Joint-State Subscriber

The subscriber is created with:

```python
self.subscription = self.create_subscription(
    JointState,
    '/joint_states',
    self.joint_state_callback,
    10
)
```

Its configuration is:

| Parameter | Value |
|---|---|
| Message type | `sensor_msgs.msg.JointState` |
| Topic | `/joint_states` |
| Callback | `self.joint_state_callback` |
| Queue depth | `10` |

The message contains the joint names and their corresponding position and velocity arrays. The controller uses these values to obtain $x$, $\dot{x}$, $\theta$, and $\dot{\theta}$.

### Force-Command Publisher

The publisher is created with:

```python
self.publisher = self.create_publisher(
    Float64,
    '/cart_force_cmd',
    10
)
```

Its configuration is:

| Parameter | Value |
|---|---|
| Message type | `std_msgs.msg.Float64` |
| Topic | `/cart_force_cmd` |
| Queue depth | `10` |

The published value represents the scalar state-space input $u=F$. The simulation interface receives this command and applies the corresponding horizontal force to `cart_rail_joint`.

---

## 5. Defining the Rigid-Body Parameters

The physical parameters used by the controller are:

```python
# System parameters
M = 3.0
m = 1.0
L = 0.5
r = 0.01
g = 9.81
```

| Variable | Value | Physical meaning |
|---|---:|---|
| `M` | $3.0\ \mathrm{kg}$ | Cart mass |
| `m` | $1.0\ \mathrm{kg}$ | Pendulum mass |
| `L` | $0.5\ \mathrm{m}$ | Total pendulum length |
| `r` | $0.01\ \mathrm{m}$ | Pendulum radius |
| `g` | $9.81\ \mathrm{m/s^2}$ | Gravitational acceleration |

These values must match the mass and geometric parameters defined in the robot model. Otherwise, the controller matrices and the simulated plant describe different systems.

### Distance from the Pivot to the Centre of Mass

The pendulum is modelled as a uniform rigid body, so its centre of mass is at its midpoint:

```python
# Distance between the pivot and the centre of mass
l = L / 2.0
```

Therefore,

$$
l=\frac{L}{2}=0.25\ \mathrm{m}
$$

The equations use $l$, not the full length $L$, for the distance from the pivot to the centre of mass.

### Moment of Inertia About the Centre of Mass

The pendulum is a solid cylinder whose rotation axis is transverse to its longitudinal axis. Its centre-of-mass moment of inertia is:

```python
# Pendulum inertia about its centre of mass
I = (m / 12.0) * (3.0 * r**2 + L**2)
```

This implements

$$
I=\frac{m}{12}\left(3r^2+L^2\right)
$$

For the parameters used in the code,

$$
I=0.0208583\ \mathrm{kg\,m^2}
$$

This term distinguishes the implemented rigid-body model from a point-mass approximation.

### Moment of Inertia About the Pivot

The parallel-axis theorem gives the pendulum inertia about the pivot:

```python
# Total pendulum inertia about the pivot
J = I + m * l**2
```

Therefore,

$$
J=I+ml^2
$$

For the implemented parameters,

$$
J=0.0833583\ \mathrm{kg\,m^2}
$$

### Common Denominator

Solving the coupled linear equations of motion in matrix form produces the determinant:

```python
# Common denominator (determinant) obtained when solving the equations of motion in matrix form
delta = (M + m) * J - (m * l)**2
```

The code variable `delta` represents the mathematical quantity

$$
\Delta=(M+m)J-(ml)^2
$$

Using $J=I+ml^2$, the same denominator can be written as

$$
\Delta=I(M+m)+Mml^2
$$

For the implemented parameters,

$$
\Delta=0.270933\ \mathrm{kg^2 \ m^2}
$$

The formulas in this document use $\Delta$, whereas the Python implementation uses the valid identifier `delta`.

---

## 6. Constructing the Rigid-Body State-Space Model

The linearized rigid-body model about the upright equilibrium is

$$
\dot{\mathbf{x}}=A\mathbf{x}+Bu
$$

where

$$
\mathbf{x}=\begin{bmatrix}x\\
\dot{x}\\
\theta\\
\dot{\theta}\end{bmatrix}
$$

and 

$$
u = F
$$

### State Matrix

The code constructs $A$ as:

```python
# State-space matrices
A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, -(m**2 * g * l**2) / delta, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, m * g * l * (M + m) / delta, 0.0]
])
```

This is the numerical implementation of

$$
A=\begin{bmatrix}0&1&0&0\\
0&0&-\dfrac{m^2gl^2}{\Delta}&0\\
0&0&0&1\\
0&0&\dfrac{mgl(M+m)}{\Delta}&0\end{bmatrix}.
$$

The rows correspond to:

$$
\dot{x}_1=x_2,
$$

$$
\dot{x}_2=-\frac{m^2gl^2}{\Delta}x_3+\frac{J}{\Delta}u,
$$

$$
\dot{x}_3=x_4,
$$

$$
\dot{x}_4=\frac{mgl(M+m)}{\Delta}x_3-\frac{ml}{\Delta}u.
$$

The first and third rows are kinematic relationships. The second and fourth rows contain the coupled rigid-body dynamics.

### Input Matrix

The code constructs $B$ as:

```python
B = np.array([
    [0.0],
    [J / delta],
    [0.0],
    [-m * l / delta]
])
```

This implements

$$
B=\begin{bmatrix}0\\
\dfrac{J}{\Delta}\\
0\\
-\dfrac{ml}{\Delta}\end{bmatrix}.
$$

Since $J=I+ml^2$, the second element is equivalent to

$$
\frac{J}{\Delta}=\frac{I+ml^2}{\Delta}.
$$

The zero elements show that the applied force does not directly change either position state. The nonzero second and fourth elements describe how the cart force changes the cart acceleration and, through dynamic coupling, the pendulum angular acceleration.

The system is underactuated because it has two degrees of freedom but only one independent actuator.

---

## 7. Defining the LQR Weighting Matrices

The controller uses:

```python
# LQR weighting matrices
Q = np.diag([
    10.0,
    1.0,
    100.0,
    1.0
])

R = np.array([
    [0.1]
])
```

The state-weighting matrix is

$$
Q=\begin{bmatrix}10&0&0&0\\
0&1&0&0\\
0&0&100&0\\
0&0&0&1\end{bmatrix}.
$$

The input-weighting matrix is

$$
R=\begin{bmatrix}0.1\end{bmatrix}.
$$

The diagonal elements of $Q$ correspond to the state ordering used by the code:

| State | Weight | Effect |
|---|---:|---|
| $x$ | $10$ | Penalizes cart-position deviation |
| $\dot{x}$ | $1$ | Penalizes cart velocity |
| $\theta$ | $100$ | Strongly penalizes pendulum-angle deviation |
| $\dot{\theta}$ | $1$ | Penalizes pendulum angular velocity |

The relatively large angle weight reflects the main objective of maintaining the pendulum near its upright equilibrium. The scalar $R=0.1$ penalizes the applied force.

For this scalar-input system, the implemented cost function can be written as

$$
\mathcal{J}=\int_0^\infty\left(\mathbf{x}^TQ\mathbf{x}+ru^2\right)\,dt,
$$

where $r=0.1$. The symbol $\mathcal{J}$ denotes the LQR cost and must not be confused with the pivot moment of inertia $J$ used in the rigid-body model.

---

## 8. Solving the CARE and Calculating the LQR Gain

The controller solves the CARE once during initialization:

```python
# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)
```

The equation solved by SciPy is

$$
A^TP+PA-PBR^{-1}B^TP+Q=0.
$$

The solution $P$ is then used to calculate the state-feedback gain:

```python
# LQR gain matrix
self.K = np.linalg.inv(R) @ B.T @ P
```

This line directly implements

$$
K=R^{-1}B^TP.
$$

The `@` operator performs matrix multiplication, and `B.T` is the transpose of $B$. The result is a $1\times4$ row matrix:

$$
K=\begin{bmatrix}k_1&k_2&k_3&k_4\end{bmatrix}.
$$

The controller logs the calculated gain:

```python
self.get_logger().info(f'K matrix: {self.K}')
```

Because $A$, $B$, $Q$, and $R$ remain constant while the node is running, neither the CARE nor $K$ needs to be recalculated inside the callback.

The calculated gain is specific to the implemented rigid-body parameters. Any change to $M$, $m$, $L$, $r$, $g$, $Q$, or $R$ produces a different gain.

---

## 9. Receiving and Identifying the Joint States

The callback starts with:

```python
def joint_state_callback(self, msg):
```

It is executed whenever a new message arrives on `/joint_states`.

The controller locates the two required joints by name:

```python
cart_index = msg.name.index('cart_rail_joint')
pendulum_index = msg.name.index('pendulum_cart_joint')
```

This avoids assuming a fixed ordering within `msg.name`. These indices are then used with the corresponding `position` and `velocity` arrays.

The current implementation assumes that:

- both joint names are present in every relevant message,
- both position values are available,
- both velocity values are available.

If a required joint name is missing, Python raises a `ValueError`. The provided controller code does not include separate exception handling for that case.

---

## 10. Extracting the State Variables

The controller reads:

```python
# State variables
x = msg.position[cart_index]
x_dot = msg.velocity[cart_index]

theta = msg.position[pendulum_index]
theta_dot = msg.velocity[pendulum_index]
```

The mapping is:

| Mathematical state | Python source | Physical meaning |
|---|---|---|
| $x$ | `msg.position[cart_index]` | Cart position |
| $\dot{x}$ | `msg.velocity[cart_index]` | Cart velocity |
| $\theta$ | `msg.position[pendulum_index]` | Pendulum angle from the upright equilibrium |
| $\dot{\theta}$ | `msg.velocity[pendulum_index]` | Pendulum angular velocity |

The angle convention used by Gazebo and the robot model must match the sign convention used to derive $A$ and $B$. In this implementation, $\theta=0$ represents the upright linearization point.

---

## 11. Constructing the State Vector

The four measurements are assembled into a NumPy column vector:

```python
# State vector
state = np.array([
    [x],
    [x_dot],
    [theta],
    [theta_dot]
])
```

This produces

$$
\mathbf{x}=\begin{bmatrix}x\\
\dot{x}\\
\theta\\
\dot{\theta}\end{bmatrix}.
$$

Its shape is $4\times1$. The ordering must remain identical to the ordering used when constructing $A$, $Q$, and $K$. Changing the state order without changing the matrices would associate each gain with the wrong physical variable.

---

## 12. Calculating the LQR Control Force

The control force is calculated with:

```python
# LQR control force
force = -self.K @ state
```

This implements the scalar control law

$$
u=-K\mathbf{x}.
$$

Expanded by state,

$$
u=-\left(k_1x+k_2\dot{x}+k_3\theta+k_4\dot{\theta}\right).
$$

Since `self.K` has shape $1\times4$ and `state` has shape $4\times1$, `force` is a $1\times1$ NumPy array.

The negative sign implements negative state feedback. The direction of the resulting response also depends on the coordinate conventions embedded in $A$, $B$, and the measured joint states.

The code does not apply force saturation, a dead band, filtering, or a swing-up controller. It implements only the linear LQR stabilization law. Therefore, it is intended to operate near the upright equilibrium where the linearized model is valid.

---

## 13. Converting and Publishing the Force

The NumPy result is converted to a ROS 2 message:

```python
# Convert the force to a ROS message
force_msg = Float64()
force_msg.data = float(force[0, 0])
```

The indexing operation `force[0, 0]` extracts the scalar from the $1\times1$ array, and `float()` converts it to a standard Python floating-point value.

The message is then published:

```python
# Publish the control force
self.publisher.publish(force_msg)
```

The publication closes the feedback path:

1. Gazebo publishes the latest joint states.
2. The callback calculates $u=-K\mathbf{x}$.
3. The node publishes the scalar force through `/cart_force_cmd`.
4. The simulation interface applies the force to the cart.
5. The simulated system evolves and produces new joint states.

Because the supplied code has no explicit saturation, the published value is the raw LQR output.

---

## 14. Starting and Stopping the Node

The entry point is:

```python
def main(args=None):
    rclpy.init(args=args)

    node = InvertedPendulumController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

The sequence is:

1. `rclpy.init(args=args)` initializes ROS 2.
2. `InvertedPendulumController()` creates the node and calculates the LQR gain.
3. `rclpy.spin(node)` keeps the node active and dispatches incoming messages to the callback.
4. `node.destroy_node()` releases the node resources after spinning stops.
5. `rclpy.shutdown()` shuts down the ROS 2 client library.

---

## 15. Theory-to-Code Mapping

| Mathematical or control concept | Implementation |
|---|---|
| Cart mass $M$ | `M = 3.0` |
| Pendulum mass $m$ | `m = 1.0` |
| Total pendulum length $L$ | `L = 0.5` |
| Pendulum radius $r$ | `r = 0.01` |
| Centre-of-mass distance $l=L/2$ | `l = L / 2.0` |
| Centre-of-mass inertia $I$ | `I = (m / 12.0) * (3.0 * r**2 + L**2)` |
| Pivot inertia $J=I+ml^2$ | `J = I + m * l**2` |
| Common denominator $\Delta=(M+m)J-(ml)^2$ | `delta = (M + m) * J - (m * l)**2` |
| State matrix $A$ | `A = np.array(...)` |
| Input matrix $B$ | `B = np.array(...)` |
| State weighting $Q$ | `Q = np.diag([10.0, 1.0, 100.0, 1.0])` |
| Input weighting $R$ | `R = np.array([[0.1]])` |
| CARE solution $P$ | `P = solve_continuous_are(A, B, Q, R)` |
| LQR gain $K$ | `self.K = np.linalg.inv(R) @ B.T @ P` |
| State vector $\mathbf{x}$ | `state` |
| Control law $u=-K\mathbf{x}$ | `force = -self.K @ state` |
| Force message | `Float64` |
| Force publication | `self.publisher.publish(force_msg)` |

---

## 16. Complete Controller Code

The complete implementation described in this document is:

```python
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

        # System parameters
        M = 3.0
        m = 1.0
        L = 0.5
        r = 0.01
        g = 9.81

        # Distance between the pivot and the centre of mass
        l = L / 2.0

        # Pendulum inertia about its centre of mass
        I = (m / 12.0) * (3.0 * r**2 + L**2)

        # Total pendulum inertia about the pivot
        J = I + m * l**2

        # Common denominator (determinant) obtained when solving the equations of motion in matrix form
        delta = (M + m) * J - (m * l)**2

        # State-space matrices
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

        # LQR weighting matrices
        Q = np.diag([
            10.0,
            1.0,
            100.0,
            1.0
        ])

        R = np.array([
            [0.1]
        ])

        # Solve the continuous-time algebraic Riccati equation
        P = solve_continuous_are(A, B, Q, R)

        # LQR gain matrix
        self.K = np.linalg.inv(R) @ B.T @ P

        self.get_logger().info(f'K matrix: {self.K}')

    def joint_state_callback(self, msg):

        cart_index = msg.name.index('cart_rail_joint')
        pendulum_index = msg.name.index('pendulum_cart_joint')

        # State variables
        x = msg.position[cart_index]
        x_dot = msg.velocity[cart_index]

        theta = msg.position[pendulum_index]
        theta_dot = msg.velocity[pendulum_index]

        # State vector
        state = np.array([
            [x],
            [x_dot],
            [theta],
            [theta_dot]
        ])

        # LQR control force
        force = -self.K @ state

        # Convert the force to a ROS message
        force_msg = Float64()
        force_msg.data = float(force[0, 0])

        # Publish the control force
        self.publisher.publish(force_msg)


def main(args=None):
    rclpy.init(args=args)

    node = InvertedPendulumController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 17. Conclusion

The controller node implements the same rigid-body model derived in the preceding documents. It includes the cylinder's centre-of-mass inertia $I$, the pivot inertia $J=I+ml^2$, and the determinant $\Delta=(M+m)J-(ml)^2$.

During initialization, the node constructs $A$, $B$, $Q$, and $R$, solves the CARE, and stores the LQR gain $K$. For every `JointState` callback, it constructs the measured state vector and calculates

$$
u=-K\mathbf{x}.
$$

The scalar result is published as a `Float64` message through `/cart_force_cmd`. This implementation therefore connects the rigid-body mathematical model, optimal controller design, ROS 2 state measurements, and Gazebo force actuation in one feedback loop.

Continue to:

[ROS 2 and Gazebo Software Architecture](06_ros2_and_gazebo_software_architecture.md)