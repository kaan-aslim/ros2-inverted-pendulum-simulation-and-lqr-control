# LQR Controller Design

# 1. Introduction to Control Systems

## 1.1 What is a Control System?

A control system is a mathematical and computational framework used to regulate the behaviour of a dynamic system.

Its objective is to automatically generate the appropriate control input so that the system behaves as desired.

Rather than allowing the system to evolve according to its natural dynamics, the controller continuously adjusts the system input to achieve a specified objective, such as maintaining stability, tracking a desired position, or rejecting external disturbances.

In this project, the control input is the horizontal force applied to the cart.

By adjusting this force continuously, the controller keeps the pendulum balanced around its unstable upright equilibrium.

---

## 1.2 Why is Control Required?

The state-space model developed in the previous chapter describes only the natural dynamics of the inverted pendulum.

Without any controller, the system simply follows the laws of motion described by

$$
\dot{\mathbf{x}}=A\mathbf{x}+B\mathbf{u}
$$

This mathematical model predicts how the cart and pendulum will move for a given input force, but it does not determine what the input force should be.

For an inverted pendulum, the upright position is an inherently unstable equilibrium.

Any small disturbance, modelling error, or measurement noise causes the pendulum to rotate away from the upright position.

Since no corrective action is applied, the pendulum eventually falls.

Therefore, a controller is required to continuously observe the current system state and calculate the force needed to maintain the upright equilibrium.

---

## 1.3 Open-Loop and Closed-Loop Control

A system operating without feedback is called an **open-loop system**.

In an open-loop system, the control input is independent of the current system state.

Once the input is applied, no information from the system output is used to modify the control action.

Consequently, disturbances and modelling uncertainties cannot be corrected automatically.

In contrast, a **closed-loop system** continuously measures the current system state and uses this information to compute a new control input.

The controller repeatedly performs the following cycle:

1. Measure the current state.
2. Compute the control input.
3. Apply the control force.
4. Observe the updated system state.
5. Repeat.

This continuous feedback process enables the controller to compensate for disturbances and stabilise the system.

<p align="center">
    <img src="images/open_and_closed_loop_system.png" alt="Open and Closed Loop Control" width="1000">
</p>

For the inverted pendulum, the measured state vector is

$$
\mathbf{x}=
\begin{bmatrix}
x\\
\dot{x}\\
\theta\\
\dot{\theta}
\end{bmatrix}
$$

The controller uses these measured states to calculate the force applied to the cart.

The resulting control input continuously changes as the system state evolves.

For this reason, the inverted pendulum is always operated as a closed-loop system.

---

# 2. Why the Inverted Pendulum Needs Control

The objective of the inverted pendulum is to maintain the pendulum in its unstable upright equilibrium while allowing the cart to move along the rail.

Unlike a hanging pendulum, which naturally returns to its stable equilibrium under gravity, an inverted pendulum behaves in the opposite manner.

Any small deviation from the upright position causes gravity to increase the pendulum angle rather than reduce it.

As a result, the pendulum rapidly falls unless a corrective action is continuously applied.

The state-space model derived in the previous chapter accurately describes the system dynamics.

However, it does not determine the force required to stabilise the pendulum.

Instead, it predicts how the system responds when a particular force is applied.

Therefore, a controller must compute the appropriate control force based on the current state of the system.

The controller continuously measures the cart position, cart velocity, pendulum angle, and pendulum angular velocity.

These measured states are then used to calculate the force applied to the cart.

By repeatedly updating the control force, the controller maintains the pendulum close to its upright equilibrium.

---

# 3. From State-Space Model to State Feedback

The linearized state-space model obtained previously is

$$
\dot{\mathbf{x}}=A\mathbf{x}+B\mathbf{u}
$$

This equation describes how the system evolves over time.

The first term,

$$
A\mathbf{x}
$$

represents the natural dynamics of the system.

The second term,

$$
B\mathbf{u}
$$

represents the effect of the external control input.

At this stage, the input

$$
\mathbf{u}
$$

is still unknown.

The purpose of the controller is to determine this control input.

One of the simplest and most effective approaches is **state feedback control**.

Instead of selecting a constant force, the control input is calculated from the current system state.

The control law is written as

$$
\mathbf{u}=-K\mathbf{x}
$$

where

- $$\mathbf{x}$$ is the state vector,
- $$K$$ is the state-feedback gain matrix,
- $$\mathbf{u}$$ is the control input.

The negative sign indicates **negative feedback**.

If the pendulum moves away from the desired equilibrium, the controller generates a force that opposes the motion and drives the system back toward equilibrium.

The remaining question is therefore:

> **How should the gain matrix \(K\) be selected?**

One possible solution is the **Linear Quadratic Regulator (LQR)**.

---

# 4. Linear Quadratic Regulator (LQR)

The Linear Quadratic Regulator (LQR) is one of the most widely used optimal control methods for linear dynamic systems.

Rather than selecting the feedback gain matrix by trial and error, LQR computes an optimal gain matrix by solving a mathematical optimisation problem.

The objective is to stabilise the system while simultaneously minimising the control effort.

Instead of asking

> "How can the pendulum be balanced?"

LQR asks

> "What is the best control action that balances the pendulum while using as little control effort as possible?"

For this reason, LQR belongs to the class of **optimal control methods**.

The optimisation is performed by minimising a mathematical performance index called the **cost function**.

---

# 5. Cost Function

The cost function used by the Linear Quadratic Regulator is

$$
J=\int_0^\infty\left(\mathbf{x}^TQ\mathbf{x}+\mathbf{u}^TR\mathbf{u}\right)dt
$$

The objective of the controller is to minimise the value of

$$
J
$$

during the entire motion of the system.

The cost function consists of two separate terms.

The first term,

$$
\mathbf{x}^TQ\mathbf{x}
$$

penalises deviations of the system states from the desired equilibrium.

Large deviations produce a larger cost.

The second term,

$$
\mathbf{u}^TR\mathbf{u}
$$

penalises the control effort.

Large control forces also increase the cost.

Consequently, the controller attempts to find a balance between

- keeping the pendulum close to the upright position,
- using a reasonable control force.

Instead of minimising only the state error or only the control effort, LQR minimises both simultaneously.

For this reason, the resulting controller is called an **optimal controller**.

---

# 6. Choosing the Weighting Matrices

The Linear Quadratic Regulator determines the optimal controller by minimising the cost function

$$
J=\int_0^\infty\left(\mathbf{x}^TQ\mathbf{x}+\mathbf{u}^TR\mathbf{u}\right)dt
$$

The weighting matrices

$$
Q
$$

and

$$
R
$$

allow the designer to specify which aspects of the system should be prioritised during optimisation.

Rather than changing the system dynamics, these matrices define how strongly different quantities are penalised in the cost function.

---

## 6.1 State Weighting Matrix (Q)

The matrix

$$
Q
$$

penalises deviations of the system states from the desired equilibrium.

For the inverted pendulum, a typical weighting matrix is

$$
Q=
\begin{bmatrix}
q_x & 0 & 0 & 0 \\
0 & q_{\dot{x}} & 0 & 0 \\
0 & 0 & q_\theta & 0 \\
0 & 0 & 0 & q_{\dot{\theta}}
\end{bmatrix}
$$

Each diagonal element determines how important a particular state is during optimisation.

A larger value means that deviations of that state produce a larger cost.

For the inverted pendulum,

- $$q_x$$ penalises cart position error.
- $$q_{\dot{x}}$$ penalises cart velocity.
- $$q_\theta$$ penalises pendulum angle.
- $$q_{\dot{\theta}}$$ penalises pendulum angular velocity.

Since maintaining the upright position is the primary objective, the pendulum angle usually receives the largest weight.

As a result, the controller prioritises keeping the pendulum balanced over maintaining the cart exactly at the origin.

---

## 6.2 Control Weighting Matrix (R)

The matrix

$$
R
$$

penalises the control effort.

For a single-input system,

$$
R=
\begin{bmatrix}
r
\end{bmatrix}
$$

A larger value of

$$
r
$$

increases the penalty associated with the applied force.

Consequently, the controller generates smaller and smoother control inputs.

Conversely, a smaller value allows the controller to apply larger forces, resulting in faster system responses.

The choice of

$$
Q
$$

and

$$
R
$$

therefore determines the trade-off between

- tracking performance,
- control effort.

---

# 7. Solving the Riccati Equation

Once the weighting matrices have been selected, the next step is to compute the optimal feedback gain matrix.

Instead of determining the gain matrix directly, the Linear Quadratic Regulator first solves the continuous-time Algebraic Riccati Equation.

The Riccati equation is

$$
A^TP+PA-PBR^{-1}B^TP+Q=0
$$

where

- $$A$$ is the system matrix,
- $$B$$ is the input matrix,
- $$Q$$ is the state weighting matrix,
- $$R$$ is the control weighting matrix,
- $$P$$ is the unknown positive-definite matrix.

Unlike the system matrices,

$$
P
$$

does not represent a physical quantity.

Instead, it is an intermediate mathematical result obtained during the optimisation process.

The matrix

$$
P
$$

contains the information required to compute the optimal feedback gain.

---

## Why is the Riccati Equation Required?

The objective of LQR is to minimise the cost function while satisfying the system dynamics.

This optimisation problem can be solved analytically.

The solution leads directly to the Algebraic Riccati Equation.

Therefore, solving the Riccati equation is equivalent to solving the optimal control problem.

Once

$$
P
$$

has been obtained, the optimal feedback gain matrix can be calculated immediately.

---

# 8. Computing the Feedback Gain Matrix

After solving the Riccati equation, the optimal feedback gain matrix is calculated as

$$
K=R^{-1}B^TP
$$

The matrix

$$
K
$$

contains one feedback gain for each state variable.

For the inverted pendulum,

$$
K=
\begin{bmatrix}
k_1 & k_2 & k_3 & k_4
\end{bmatrix}
$$

Each gain determines how strongly the corresponding state contributes to the applied control force.

The control input is therefore calculated as

$$
u=-Kx
$$

Substituting the gain matrix gives

$$
u=-(k_1x+k_2\dot{x}+k_3\theta+k_4\dot{\theta})
$$

This equation shows that the control force depends on every state of the system.

Rather than considering only the pendulum angle, LQR simultaneously accounts for

- cart position,
- cart velocity,
- pendulum angle,
- pendulum angular velocity.

This complete use of the system state is one of the main reasons why LQR provides excellent stabilisation performance.

---

# 9. Closed-Loop System Dynamics

Substituting the state-feedback control law

$$
u=-Kx
$$

into the state-space equation

$$
\dot{x}=Ax+Bu
$$

gives

$$
\dot{x}=Ax+B(-Kx)
$$

Rearranging,

$$
\dot{x}=(A-BK)x
$$

The matrix

$$
A-BK
$$

is called the **closed-loop system matrix**.

Unlike the original system matrix,

$$
A
$$

the closed-loop matrix includes the effect of the controller.

The eigenvalues of

$$
A-BK
$$

determine the stability of the controlled system.

If all eigenvalues lie in the left-half complex plane, the closed-loop system is asymptotically stable.

Consequently, the pendulum returns to its upright equilibrium after small disturbances.

---

# 10. Control Loop Implementation

The LQR controller is implemented as a continuous feedback loop.

The controller repeatedly performs the following sequence.

1. Read the current joint states from Gazebo.
2. Construct the state vector

$$
x=
\begin{bmatrix}
x\\
\dot{x}\\
\theta\\
\dot{\theta}
\end{bmatrix}
$$

3. Compute the control input

$$
u=-Kx
$$

4. Publish the calculated force to the cart.
5. Receive the updated system states.
6. Repeat the process throughout the simulation.

This feedback cycle continuously compensates for disturbances and maintains the pendulum close to its unstable upright equilibrium.

---

# 11. Summary

In this chapter, the linearised state-space model was transformed into a closed-loop control system using the Linear Quadratic Regulator.

The controller computes the optimal feedback gain matrix by minimising a quadratic cost function that balances state accuracy and control effort.

The resulting control law

$$
u=-Kx
$$

continuously adjusts the cart force according to the current system state.

Substituting this control law into the state-space model produces the closed-loop system

$$
\dot{x}=(A-BK)x
$$

This equation forms the mathematical foundation of the control node implemented in the following software architecture.

