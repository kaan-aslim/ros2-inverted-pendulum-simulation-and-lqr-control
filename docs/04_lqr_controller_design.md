# LQR Controller Design

# Introduction to Control Systems

## 1. What Is a Control System?

A control system is a mathematical and computational framework used to regulate the behaviour of a dynamic system.

Its objective is to generate an appropriate control input automatically so that the system behaves as desired.

Rather than allowing the system to evolve according to its natural dynamics, the controller continuously adjusts the system input to achieve an objective such as maintaining stability, tracking a desired position, or rejecting external disturbances.

In this project, the control input is the horizontal force applied to the cart. By continuously adjusting this force, the controller keeps the rigid pendulum balanced around its unstable upright equilibrium.

---

## 2. Why Is Control Required?

The state-space model developed in the previous chapter describes the natural dynamics of the cart and rigid pendulum:

$$
\dot{\mathbf{x}}=A\mathbf{x}+Bu
$$

This model predicts how the cart and pendulum move for a given input force, but it does not determine what that force should be.

The upright position of an inverted pendulum is an inherently unstable equilibrium. Any small disturbance, modelling error, or measurement noise causes the pendulum to rotate away from this position. Without corrective action, the pendulum eventually falls.

A controller is therefore required to observe the current system state continuously and calculate the force needed to maintain the upright equilibrium.

---

## 3. Open-Loop and Closed-Loop Control

A system operating without feedback is called an **open-loop system**. Its control input is independent of the current system state, so disturbances and modelling uncertainties cannot be corrected automatically.

In contrast, a **closed-loop system** continuously measures the current system state and uses this information to calculate a new control input. The controller repeatedly performs the following cycle:

1. Measure the current state.
2. Calculate the control input.
3. Apply the control force.
4. Observe the updated system state.
5. Repeat.

This continuous feedback process enables the controller to compensate for disturbances and stabilise the system.

<p align="center">
    <img src="images/open_and_closed_loop_system.png" alt="Open and Closed Loop Control" width="1000">
</p>

For the inverted pendulum, the measured state vector is

$$
\mathbf{x}=\begin{bmatrix}x\\
\dot{x}\\
\theta\\
\dot{\theta}\end{bmatrix}
$$

where $x$ is the cart position and $\theta$ is the pendulum angle measured from the upright equilibrium. The controller uses these measured states to calculate the horizontal force applied to the cart.

---

## 4. Why the Inverted Pendulum Needs Control

The control objective is to maintain the rigid pendulum at its unstable upright equilibrium while allowing the cart to move along the rail.

Unlike a hanging pendulum, which naturally returns to its stable equilibrium under gravity, an inverted pendulum moves farther away from its equilibrium after a small angular deviation. The rigid body's mass distribution also affects this motion through its centre-of-mass moment of inertia $I$.

The state-space model predicts the response of the system to an applied force. It does not, by itself, calculate the force required for stabilisation. The controller must therefore use the cart position, cart velocity, pendulum angle, and pendulum angular velocity to determine the required force at every control step.

---

## 5. Rigid-Body State-Space Model

The LQR controller must be designed using the same rigid-body model implemented in the control node. The physical parameters are:

| Symbol | Description |
|---|---|
| $M$ | Cart mass |
| $m$ | Pendulum mass |
| $l$ | Distance from the pivot to the pendulum's centre of mass |
| $I$ | Pendulum moment of inertia about its centre of mass |
| $g$ | Gravitational acceleration |

The pendulum moment of inertia about the pivot is

$$
J=I+ml^2
$$

The common denominator obtained from the determinant of the rigid-body mass matrix is

$$
\Delta=(M+m)J-(ml)^2
$$

Substituting $J=I+ml^2$ gives the equivalent expression

$$
\Delta=I(M+m)+Mml^2
$$

The symbol $\Delta$ is the same quantity denoted by $p$ in some derivations. Therefore, the implementation

```python
J = I + m * l**2
delta = (M + m) * J - (m * l)**2
```

is consistent with the rigid-body equations.

For the state vector $\mathbf{x}=\begin{bmatrix}x&\dot{x}&\theta&\dot{\theta}\end{bmatrix}^T$ and the scalar input $u=F$, the linearised model about the upright equilibrium is

$$
\dot{\mathbf{x}}=A\mathbf{x}+Bu
$$

with

$$
A=\begin{bmatrix}0&1&0&0\\
0&0&-\dfrac{m^2gl^2}{\Delta}&0\\
0&0&0&1\\
0&0&\dfrac{mgl(M+m)}{\Delta}&0\end{bmatrix}
$$

and

$$
B=\begin{bmatrix}0\\
\dfrac{I+ml^2}{\Delta}\\
0\\
-\dfrac{ml}{\Delta}\end{bmatrix}.
$$

The inertia $I$ appears explicitly in both $\Delta$ and $B$. Consequently, replacing the rigid pendulum with a point-mass approximation changes the system matrices, the Riccati equation solution, and the resulting LQR gain.

Before designing the controller, the pair $(A,B)$ must be controllable:

$$
\mathcal{C}=\begin{bmatrix}B&AB&A^2B&A^3B\end{bmatrix}
$$

For this four-state system to be controllable, the controllability matrix must satisfy

$$
\mathrm{rank}(\mathcal{C})=4.
$$

---

## 6. From State-Space Model to State Feedback

The term $A\mathbf{x}$ represents the natural dynamics of the rigid-body system, whereas $Bu$ represents the effect of the external force.

In state-feedback control, the scalar input is calculated from the current state:

$$
u=-K\mathbf{x}
$$

where $\mathbf{x}$ is the state vector, $K$ is the state-feedback gain matrix, and $u$ is the horizontal force applied to the cart. The negative sign denotes negative feedback.

The remaining design question is:

> **How should the gain matrix $K$ be selected?**

One effective solution is the **Linear Quadratic Regulator (LQR)**.

---

# Linear Quadratic Regulator

The Linear Quadratic Regulator is a widely used optimal-control method for linear dynamic systems. Rather than selecting the feedback gains by trial and error, LQR calculates an optimal gain matrix by solving a mathematical optimisation problem.

The objective is to stabilise the rigid pendulum while balancing state regulation against control effort.

---

## 7. Cost Function

The infinite-horizon continuous-time LQR cost function is

$$
\mathcal{J}=\int_0^\infty\left(\mathbf{x}^TQ\mathbf{x}+ru^2\right)\,dt.
$$

The term $\mathbf{x}^TQ\mathbf{x}$ penalises deviations from the desired equilibrium, whereas $ru^2$ penalises the applied cart force.

The designer defines the control priorities through the weighting matrices $Q$ and $R$. Once these matrices have been selected, a numerical solver determines the controller that minimises $\mathcal{J}$.

---

## 8. Choosing the Weighting Matrices

### State-Weighting Matrix $Q$

For the state ordering used in this project, a typical diagonal state-weighting matrix is

$$
Q=\begin{bmatrix}q_x&0&0&0\\
0&q_{\dot{x}}&0&0\\
0&0&q_\theta&0\\
0&0&0&q_{\dot{\theta}}\end{bmatrix}.
$$

Each diagonal element penalises the corresponding state:

- $q_x$ penalises cart-position error.
- $q_{\dot{x}}$ penalises cart velocity.
- $q_\theta$ penalises pendulum-angle error.
- $q_{\dot{\theta}}$ penalises pendulum angular velocity.

Because maintaining the upright orientation is the primary objective, the pendulum angle usually receives a relatively large weight.

### Input-Weighting Matrix $R$

The inverted-pendulum model has one control input, so

$$
R=\begin{bmatrix}r\end{bmatrix},\qquad r>0.
$$

A larger $r$ penalises the applied force more strongly and generally produces smaller, smoother control inputs. A smaller $r$ permits larger forces and generally produces a faster, more aggressive response.

The choices of $Q$ and $R$ do not change the rigid-body plant model. They define the trade-off between state regulation and control effort.

---

## 9. Solving the Riccati Equation

After selecting $Q$ and $R$, the continuous-time algebraic Riccati equation (CARE) is solved:

$$
A^TP+PA-PBR^{-1}B^TP+Q=0.
$$

Here, $A$ and $B$ are the rigid-body system matrices, $Q$ is the state-weighting matrix, $R$ is the input-weighting matrix, and $P$ is the symmetric positive-semidefinite solution used to calculate the optimal feedback gain.

The matrix $P$ is an optimisation result rather than a physical parameter. In practice, a scientific computing library solves the CARE numerically.

Because $A$ and $B$ contain the centre-of-mass inertia $I$, the resulting $P$ and $K$ are specific to the rigid-body model and its parameter values.

---

## 10. Computing the Feedback Gain Matrix

After solving the CARE, the optimal feedback gain is

$$
K=R^{-1}B^TP.
$$

For this single-input, four-state system,

$$
K=\begin{bmatrix}k_1&k_2&k_3&k_4\end{bmatrix}.
$$

The applied force is therefore

$$
u=-K\mathbf{x}=-\left(k_1x+k_2\dot{x}+k_3\theta+k_4\dot{\theta}\right).
$$

Each gain determines how strongly its corresponding state contributes to the applied cart force. Since the gains are calculated from the rigid-body matrices, they should be recalculated whenever $M$, $m$, $l$, $I$, $g$, $Q$, or $R$ changes.

---

## 11. Closed-Loop System Dynamics

Substituting the state-feedback law into the state-space model gives

$$
\dot{\mathbf{x}}=A\mathbf{x}+B(-K\mathbf{x}).
$$

Therefore,

$$
\dot{\mathbf{x}}=(A-BK)\mathbf{x}.
$$

The matrix $A-BK$ is the **closed-loop system matrix**. Its eigenvalues determine the local stability and dynamic response of the controlled system.

If all closed-loop eigenvalues have strictly negative real parts, the linearised closed-loop system is asymptotically stable. The controller can then return the pendulum to the upright equilibrium after sufficiently small disturbances, provided that actuator limits and the validity range of the linearised model are respected.

---

## 12. Control-Loop Implementation

The LQR controller is implemented as a continuous feedback loop:

1. Read the current joint states from Gazebo.
2. Construct the state vector:

$$
\mathbf{x}=\begin{bmatrix}x\\
\dot{x}\\
\theta\\
\dot{\theta}\end{bmatrix}.
$$

3. Calculate the control force:

$$
u=-K\mathbf{x}.
$$

4. Apply the required actuator force limit.
5. Publish the calculated force to the cart.
6. Receive the updated system state and repeat.

The parameter values and sign conventions used to generate $K$ must match those used by the control node. In particular, the controller must use the rigid-body pivot inertia $J=I+ml^2$ and the same definition of positive pendulum angle.

---

## 13. Summary

This chapter used the linearised rigid-body state-space model to design a continuous-time LQR controller. Unlike a point-mass approximation, this model includes the pendulum's centre-of-mass moment of inertia $I$ and the pivot inertia $J=I+ml^2$.

The rigid-body matrices $A$ and $B$ are used in the CARE, and the resulting optimal gain defines the control law

$$
u=-K\mathbf{x}.
$$

The corresponding closed-loop dynamics are

$$
\dot{\mathbf{x}}=(A-BK)\mathbf{x}.
$$

This model is consistent with the rigid-body equations implemented in the control node and is suitable for systems in which rotational inertia is significant, including humanoid mechanisms and reaction-wheel-based platforms.

Continue to:

[LQR Controller Node Software Implementation](05_lqr_controller_node_software_implementation.md)