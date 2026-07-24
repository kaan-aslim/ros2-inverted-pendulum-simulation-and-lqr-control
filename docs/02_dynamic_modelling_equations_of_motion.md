# Dynamic Modelling: Equations of Motion

## Purpose

The physical modelling stage defined the components, degrees of freedom, coordinate system, parameters, and assumptions of the inverted pendulum system.

The next step is to describe how the system moves under the effect of forces.

This requires the development of a dynamic model.

The dynamic model establishes the mathematical relationship between:

- the horizontal force applied to the cart,
- the translational motion of the cart,
- the rotational motion of the pendulum,
- gravity,
- inertia,
- and the interaction between the cart and the pendulum.

In this project, the nonlinear equations of motion are derived using two different approaches:

1. Newton–Euler mechanics
2. Lagrangian mechanics

Both methods describe the same physical system and lead to the same nonlinear equations of motion.

These equations later form the basis for:

- linearisation,
- state-space representation,
- LQR controller design,
- and controller implementation in ROS 2.

---

## Dynamic Modelling Workflow

The dynamic modelling process used in this project follows the sequence below:

```text
Physical System Model
        ↓
Define Generalised Coordinates
        ↓
Identify Velocities and Accelerations
        ↓
Identify External and Interaction Forces
        ↓
Apply Newton–Euler Equations
        ↓
Apply Lagrange's Equations
        ↓
Obtain Nonlinear Equations of Motion
        ↓
Prepare the Model for Linearisation
```

The two derivation methods are presented separately so that the physical meaning of the equations can be understood from both force-based and energy-based perspectives.

---

## 1. System Definition

The inverted pendulum system consists of:

- a cart moving horizontally along a fixed rail,
- a pendulum rotating about a pivot attached to the cart,
- and a horizontal control force applied to the cart.

The system has two degrees of freedom:

- cart translation,
- pendulum rotation.

The generalised coordinates are defined as

$$
q =
\begin{bmatrix}
x \\
\theta
\end{bmatrix}
$$

where:

- $x$ is the horizontal position of the cart,
- $\theta$ is the angular displacement of the pendulum from the upward vertical position.

Their time derivatives are

$$
\dot{q} =
\begin{bmatrix}
\dot{x} \\
\dot{\theta}
\end{bmatrix}
$$

and

$$
\ddot{q} =
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}.
$$

The horizontal force applied to the cart is represented by

$$
F.
$$

---

## 2. Sign Convention

A consistent sign convention must be defined before deriving the equations.

For this project:

- positive cart displacement $x$ is directed to the right,
- positive cart velocity $\dot{x}$ is directed to the right,
- positive force $F$ acts to the right,
- the pendulum angle $\theta$ is measured from the upward vertical position,
- positive $\theta$ corresponds to the selected positive rotational direction.

The upright equilibrium position is therefore

$$
\theta = 0.
$$

This definition becomes particularly important during linearisation and state-space modelling.

---

## 3. Dynamic Model Parameters

The parameters used in the derivation are:

| Symbol | Description | Unit |
|---------|-------------|------|
| $M$ | Cart mass | kg |
| $m$ | Pendulum mass | kg |
| $L$ | Total pendulum length | m |
| $l$ | Distance from the pivot to the pendulum centre of mass | m |
| $I$ | Pendulum moment of inertia about its centre of mass | kg·m² |
| $g$ | Gravitational acceleration | m/s² |
| $F$ | Horizontal force applied to the cart | N |
| $x$ | Cart position | m |
| $\theta$ | Pendulum angular displacement | rad |

For the project model:

| Parameter | Value |
|-----------|-------|
| $M$ | 3.0 kg |
| $m$ | 1.0 kg |
| $L$ | 0.5 m |
| $l$ | 0.25 m |
| $g$ | 9.81 m/s² |

The analytical controller model uses the common point-mass approximation in which the pendulum mass is considered to be concentrated at its centre of mass.

Under this approximation, the separate rotational inertia term is neglected in the simplified equations.

A more complete rigid-body model can retain the pendulum inertia $I$.

---

# Newton–Euler Derivation

## 4. Overview of the Newton–Euler Method

The Newton–Euler method develops the equations of motion directly from forces, accelerations, and moments.

It combines:

- Newton's second law for translational motion,
- Euler's rotational equation for rotational motion.

Newton's second law is

$$
\sum F = ma.
$$

Euler's rotational equation is

$$
\sum \tau = I\alpha.
$$

For the inverted pendulum system, the cart and the pendulum are first analysed as separate bodies.

The interaction forces between them are introduced at the pivot and later eliminated to obtain equations containing only the generalised coordinates $x$ and $\theta$.

---

## 5. Position of the Pendulum Centre of Mass

The pendulum pivot moves together with the cart.

The horizontal and vertical coordinates of the pendulum centre of mass are

$$
x_p = x + l\sin\theta
$$

and

$$
y_p = l\cos\theta.
$$

The vertical coordinate is measured relative to the pivot level.

When

$$
\theta = 0,
$$

the pendulum is upright and its centre of mass is directly above the pivot.

---

## 6. Pendulum Centre-of-Mass Velocity

Differentiating the position equations gives the velocity components.

The horizontal velocity is

$$
\dot{x}_p
=
\dot{x}
+
l\dot{\theta}\cos\theta.
$$

The vertical velocity is

$$
\dot{y}_p
=
-l\dot{\theta}\sin\theta.
$$

These equations show that the pendulum centre of mass has two sources of motion:

- translation caused by the cart,
- rotation about the pivot.

---

## 7. Pendulum Centre-of-Mass Acceleration

Differentiating once more gives the acceleration components.

The horizontal acceleration is

$$
\ddot{x}_p
=
\ddot{x}
+
l\ddot{\theta}\cos\theta
-
l\dot{\theta}^{2}\sin\theta.
$$

The vertical acceleration is

$$
\ddot{y}_p
=
-l\ddot{\theta}\sin\theta
-
l\dot{\theta}^{2}\cos\theta.
$$

The horizontal acceleration contains three terms:

$$
\ddot{x}
$$

from cart translation,

$$
l\ddot{\theta}\cos\theta
$$

from angular acceleration,

and

$$
-l\dot{\theta}^{2}\sin\theta
$$

from centripetal acceleration.

---

## 8. Horizontal Force Balance for the Pendulum

Let the horizontal interaction force exerted by the cart on the pendulum be represented by $H$.

Applying Newton's second law in the horizontal direction gives

$$
H = m\ddot{x}_p.
$$

Substituting the horizontal acceleration of the pendulum centre of mass:

$$
H
=
m
\left(
\ddot{x}
+
l\ddot{\theta}\cos\theta
-
l\dot{\theta}^{2}\sin\theta
\right).
$$

Therefore,

$$
H
=
m\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta.
$$

---

## 9. Horizontal Force Balance for the Cart

The cart is acted on by:

- the applied control force $F$,
- the horizontal interaction force from the pendulum.

Applying Newton's second law to the cart gives

$$
F - H = M\ddot{x}.
$$

Substituting the expression for $H$:

$$
F
-
\left(
m\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
\right)
=
M\ddot{x}.
$$

Rearranging the terms gives

$$
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
=
F.
$$

This is the first nonlinear equation of motion.

It describes the horizontal dynamics of the complete cart–pendulum system.

---

## 10. Physical Meaning of the First Equation

The first equation is

$$
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
=
F.
$$

Each term has a physical meaning.

### Combined Translational Inertia

$$
(M+m)\ddot{x}
$$

represents the force required to accelerate both the cart and the pendulum mass horizontally.

### Angular-Acceleration Coupling

$$
ml\ddot{\theta}\cos\theta
$$

represents the horizontal force contribution produced by pendulum angular acceleration.

### Centripetal Coupling

$$
-ml\dot{\theta}^{2}\sin\theta
$$

represents the nonlinear horizontal force caused by pendulum rotation.

### Applied Control Force

$$
F
$$

is the external horizontal force used to control the system.

The equation demonstrates that cart motion and pendulum motion are dynamically coupled.

The cart cannot be analysed independently from the pendulum.

---

## 11. Tangential Dynamics of the Pendulum

The motion of the pendulum can be examined along the direction tangent to its circular path.

The tangential acceleration caused by pendulum rotation is

$$
l\ddot{\theta}.
$$

Because the pivot itself accelerates horizontally with the cart, the cart acceleration also contributes to the pendulum's tangential acceleration.

The tangential component of cart acceleration is

$$
\ddot{x}\cos\theta.
$$

The tangential component of gravitational acceleration is

$$
-g\sin\theta
$$

under the sign convention used in this project.

Combining these components gives

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

This is the second nonlinear equation of motion for the simplified point-mass pendulum model.

---

## 12. Physical Meaning of the Second Equation

The second equation is

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

### Pendulum Angular Acceleration

$$
l\ddot{\theta}
$$

represents the tangential acceleration caused by rotation of the pendulum.

### Cart–Pendulum Coupling

$$
\ddot{x}\cos\theta
$$

represents the tangential effect of cart acceleration on the pendulum.

This term is the mechanism through which the cart controls the pendulum.

There is no actuator directly applying torque to the pendulum joint.

Instead, the pendulum is stabilised indirectly by accelerating the cart.

### Gravitational Effect

$$
-g\sin\theta
$$

represents the tangential component of gravity.

Near the upright position, gravity drives the pendulum away from equilibrium, making the upright configuration naturally unstable.

---

## 13. Rigid-Body Form of the Pendulum Equation

If the rotational inertia of the pendulum is retained, the rotational equation becomes

$$
\left(I + ml^2\right)\ddot{\theta}
+
ml\ddot{x}\cos\theta
-
mgl\sin\theta
=
0.
$$

Dividing by $ml$ gives

$$
\left(
l+\frac{I}{ml}
\right)\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

The simplified project equation

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0
$$

is obtained when the separate inertia term $I$ is neglected.

This simplification is widely used during introductory controller development and produces the state-space model used later in this project.

---

# Lagrangian Derivation

## 14. Overview of the Lagrangian Method

The Lagrangian method derives the equations of motion from energy rather than directly balancing individual forces.

The Lagrangian is defined as

$$
\mathcal{L} = T - V
$$

where:

- $T$ is the total kinetic energy,
- $V$ is the total potential energy.

For each generalised coordinate $q_i$, Lagrange's equation is

$$
\frac{d}{dt}
\left(
\frac{\partial \mathcal{L}}{\partial \dot{q}_i}
\right)
-
\frac{\partial \mathcal{L}}{\partial q_i}
=
Q_i
$$

where $Q_i$ is the generalised external force associated with $q_i$.

For this system:

$$
q_1 = x
$$

and

$$
q_2 = \theta.
$$

The corresponding generalised forces are

$$
Q_x = F
$$

and

$$
Q_\theta = 0.
$$

There is no direct actuator torque applied to the pendulum.

---

## 15. Kinetic Energy of the Cart

The cart moves only in the horizontal direction.

Its kinetic energy is

$$
T_c
=
\frac{1}{2}M\dot{x}^{2}.
$$

---

## 16. Kinetic Energy of the Pendulum

The velocity components of the pendulum centre of mass are

$$
\dot{x}_p
=
\dot{x}
+
l\dot{\theta}\cos\theta
$$

and

$$
\dot{y}_p
=
-l\dot{\theta}\sin\theta.
$$

The squared centre-of-mass velocity is

$$
v_p^2
=
\dot{x}_p^2
+
\dot{y}_p^2.
$$

Substituting the velocity components:

$$
v_p^2
=
\left(
\dot{x}
+
l\dot{\theta}\cos\theta
\right)^2
+
\left(
-l\dot{\theta}\sin\theta
\right)^2.
$$

Expanding:

$$
v_p^2
=
\dot{x}^2
+
2l\dot{x}\dot{\theta}\cos\theta
+
l^2\dot{\theta}^2\cos^2\theta
+
l^2\dot{\theta}^2\sin^2\theta.
$$

Using

$$
\sin^2\theta+\cos^2\theta=1,
$$

the expression becomes

$$
v_p^2
=
\dot{x}^2
+
2l\dot{x}\dot{\theta}\cos\theta
+
l^2\dot{\theta}^2.
$$

The translational kinetic energy of the pendulum is therefore

$$
T_{p,\text{trans}}
=
\frac{1}{2}m
\left(
\dot{x}^2
+
2l\dot{x}\dot{\theta}\cos\theta
+
l^2\dot{\theta}^2
\right).
$$

If the rotational inertia of the pendulum is included, its rotational kinetic energy is

$$
T_{p,\text{rot}}
=
\frac{1}{2}I\dot{\theta}^2.
$$

The total pendulum kinetic energy is

$$
T_p
=
\frac{1}{2}m
\left(
\dot{x}^2
+
2l\dot{x}\dot{\theta}\cos\theta
+
l^2\dot{\theta}^2
\right)
+
\frac{1}{2}I\dot{\theta}^2.
$$

---

## 17. Total Kinetic Energy

The total kinetic energy of the system is

$$
T = T_c + T_p.
$$

Therefore,

$$
T
=
\frac{1}{2}M\dot{x}^2
+
\frac{1}{2}m
\left(
\dot{x}^2
+
2l\dot{x}\dot{\theta}\cos\theta
+
l^2\dot{\theta}^2
\right)
+
\frac{1}{2}I\dot{\theta}^2.
$$

Collecting terms:

$$
T
=
\frac{1}{2}(M+m)\dot{x}^2
+
ml\dot{x}\dot{\theta}\cos\theta
+
\frac{1}{2}
\left(
ml^2+I
\right)
\dot{\theta}^2.
$$

For the simplified point-mass model, $I$ is neglected:

$$
T
=
\frac{1}{2}(M+m)\dot{x}^2
+
ml\dot{x}\dot{\theta}\cos\theta
+
\frac{1}{2}ml^2\dot{\theta}^2.
$$

---

## 18. Potential Energy of the Pendulum

Only the pendulum contributes gravitational potential energy.

The vertical position of the pendulum centre of mass is

$$
y_p = l\cos\theta.
$$

The potential energy is therefore

$$
V = mgy_p.
$$

Thus,

$$
V = mgl\cos\theta.
$$

The absolute zero level of potential energy is arbitrary.

Only derivatives of the potential energy appear in the equations of motion.

---

## 19. Lagrangian of the System

The Lagrangian is

$$
\mathcal{L}=T-V.
$$

For the rigid-body model:

$$
\mathcal{L}
=
\frac{1}{2}(M+m)\dot{x}^2
+
ml\dot{x}\dot{\theta}\cos\theta
+
\frac{1}{2}
\left(
ml^2+I
\right)
\dot{\theta}^2
-
mgl\cos\theta.
$$

For the simplified point-mass model:

$$
\mathcal{L}
=
\frac{1}{2}(M+m)\dot{x}^2
+
ml\dot{x}\dot{\theta}\cos\theta
+
\frac{1}{2}ml^2\dot{\theta}^2
-
mgl\cos\theta.
$$

---

## 20. Lagrange Equation for the Cart Coordinate

For the cart coordinate $x$:

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}{\partial\dot{x}}
\right)
-
\frac{\partial\mathcal{L}}{\partial x}
=
F.
$$

First,

$$
\frac{\partial\mathcal{L}}{\partial\dot{x}}
=
(M+m)\dot{x}
+
ml\dot{\theta}\cos\theta.
$$

Taking the time derivative:

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}{\partial\dot{x}}
\right)
=
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta.
$$

The Lagrangian does not explicitly depend on $x$, therefore

$$
\frac{\partial\mathcal{L}}{\partial x}=0.
$$

Substituting into Lagrange's equation:

$$
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
=
F.
$$

This is the same horizontal equation obtained using the Newton–Euler method.

---

## 21. Lagrange Equation for the Pendulum Coordinate

For the pendulum coordinate $\theta$:

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}{\partial\dot{\theta}}
\right)
-
\frac{\partial\mathcal{L}}{\partial\theta}
=
0.
$$

First,

$$
\frac{\partial\mathcal{L}}{\partial\dot{\theta}}
=
ml\dot{x}\cos\theta
+
\left(
ml^2+I
\right)\dot{\theta}.
$$

Taking the time derivative:

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}{\partial\dot{\theta}}
\right)
=
ml\ddot{x}\cos\theta
-
ml\dot{x}\dot{\theta}\sin\theta
+
\left(
ml^2+I
\right)\ddot{\theta}.
$$

Next,

$$
\frac{\partial\mathcal{L}}{\partial\theta}
=
-ml\dot{x}\dot{\theta}\sin\theta
+
mgl\sin\theta.
$$

Substituting into Lagrange's equation:

$$
ml\ddot{x}\cos\theta
-
ml\dot{x}\dot{\theta}\sin\theta
+
\left(
ml^2+I
\right)\ddot{\theta}
-
\left(
-ml\dot{x}\dot{\theta}\sin\theta
+
mgl\sin\theta
\right)
=
0.
$$

The velocity-coupling terms cancel:

$$
-ml\dot{x}\dot{\theta}\sin\theta
+
ml\dot{x}\dot{\theta}\sin\theta
=
0.
$$

The equation becomes

$$
\left(
ml^2+I
\right)\ddot{\theta}
+
ml\ddot{x}\cos\theta
-
mgl\sin\theta
=
0.
$$

Dividing by $ml$:

$$
\left(
l+\frac{I}{ml}
\right)\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

For the simplified point-mass model, $I=0$:

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

This is the same pendulum equation obtained using the Newton–Euler method.

---

# Final Nonlinear Equations of Motion

## 22. Equations Used in This Project

The simplified nonlinear dynamic model used for controller development is

$$
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
=
F
$$

and

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

These equations describe the coupled nonlinear motion of the cart and pendulum.

The first equation represents horizontal translation of the complete system.

The second equation represents rotational motion of the pendulum.

---

## 23. Matrix Form of the Nonlinear Model

The equations can also be written in a compact coupled form.

Starting from

$$
(M+m)\ddot{x}
+
ml\cos\theta\,\ddot{\theta}
=
F
+
ml\dot{\theta}^{2}\sin\theta
$$

and

$$
\cos\theta\,\ddot{x}
+
l\ddot{\theta}
=
g\sin\theta,
$$

the acceleration terms can be grouped as

$$
\begin{bmatrix}
M+m & ml\cos\theta \\
\cos\theta & l
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
F+ml\dot{\theta}^{2}\sin\theta \\
g\sin\theta
\end{bmatrix}.
$$

This form shows that $\ddot{x}$ and $\ddot{\theta}$ must be solved together.

The cart acceleration depends on the pendulum acceleration, and the pendulum acceleration depends on the cart acceleration.

This coupling is the central feature of the inverted pendulum dynamics.

---

## 24. Solving for the Accelerations

From the pendulum equation:

$$
l\ddot{\theta}
=
g\sin\theta
-
\ddot{x}\cos\theta.
$$

Therefore,

$$
\ddot{\theta}
=
\frac{
g\sin\theta
-
\ddot{x}\cos\theta
}{l}.
$$

Substituting this expression into the cart equation gives an explicit expression for cart acceleration.

After rearrangement:

$$
\ddot{x}
=
\frac{
F
+
ml\dot{\theta}^{2}\sin\theta
-
mg\sin\theta\cos\theta
}{
M+m-m\cos^{2}\theta
}.
$$

Since

$$
M+m-m\cos^{2}\theta
=
M+m\sin^{2}\theta,
$$

the cart acceleration can be written as

$$
\ddot{x}
=
\frac{
F
+
ml\dot{\theta}^{2}\sin\theta
-
mg\sin\theta\cos\theta
}{
M+m\sin^{2}\theta
}.
$$

The angular acceleration is then

$$
\ddot{\theta}
=
\frac{
g\sin\theta
-
\ddot{x}\cos\theta
}{l}.
$$

These explicit acceleration equations are useful for:

- nonlinear simulation,
- numerical integration,
- model verification,
- and future swing-up controller development.

---

## 25. Why the Model Is Nonlinear

The equations contain nonlinear terms such as

$$
\sin\theta,
$$

$$
\cos\theta,
$$

and

$$
\dot{\theta}^{2}\sin\theta.
$$

These terms make the system nonlinear.

For example:

- the influence of gravity changes with pendulum angle,
- the coupling between cart and pendulum changes with $\cos\theta$,
- the centripetal term depends on the square of angular velocity.

A linear state-space controller cannot be designed directly from these equations without first linearising them around an operating point.

For this project, the selected operating point is the upright equilibrium:

$$
x=0,
$$

$$
\dot{x}=0,
$$

$$
\theta=0,
$$

$$
\dot{\theta}=0.
$$

---

## 26. How the Equations Are Used in the Project

The nonlinear equations are not directly inserted into the LQR controller.

Instead, they define the physical dynamics from which the controller model is developed.

The project uses the equations through the following sequence:

```text
Nonlinear Equations of Motion
        ↓
Select Upright Equilibrium
        ↓
Apply Small-Angle Approximations
        ↓
Obtain Linear Differential Equations
        ↓
Define State Variables
        ↓
Construct A and B Matrices
        ↓
Calculate the LQR Gain Matrix
        ↓
Implement u = -Kx in the ROS 2 Control Node
```

The nonlinear equations are also useful for checking whether the signs, parameters, and coupling relationships used in the controller are physically correct.

---

## 27. Relationship Between the Mathematical Model and Gazebo

The analytical model represents the system using two generalised coordinates:

$$
x
$$

and

$$
\theta.
$$

In the Gazebo model, these quantities correspond to:

| Mathematical Quantity | Gazebo / URDF Representation |
|-----------------------|------------------------------|
| $x$ | Position of `cart_rail_joint` |
| $\dot{x}$ | Velocity of `cart_rail_joint` |
| $\theta$ | Position of `pendulum_cart_joint` |
| $\dot{\theta}$ | Velocity of `pendulum_cart_joint` |
| $F$ | Force command applied to `cart_rail_joint` |
| $M$ | Mass of `cart_link` |
| $m$ | Mass of `pendulum_link` |
| $l$ | Pivot-to-centre-of-mass distance |
| $g$ | Gravity configured in the Gazebo world |

The ROS 2 controller reads the joint states, forms the state vector, calculates the required control force, and applies that force to the cart joint.

The dynamic equations explain why this horizontal force can influence both cart position and pendulum angle.

---

## 28. Newton–Euler and Lagrange Comparison

Both methods produce the same equations, but they emphasise different aspects of the system.

| Newton–Euler Method | Lagrangian Method |
|---------------------|-------------------|
| Based on force and moment balances | Based on kinetic and potential energy |
| Makes interaction forces visible | Eliminates many internal forces automatically |
| Provides direct physical interpretation | Often more systematic for multi-body systems |
| Requires separate free-body diagrams | Requires position, velocity, and energy expressions |
| Useful for understanding forces | Useful for deriving complex robotic dynamics |

For this project:

- Newton–Euler mechanics explains the physical origin of each force term.
- Lagrangian mechanics provides a structured verification of the same model.
- Agreement between both derivations increases confidence in the final equations.

---

## 29. Important Modelling Notes

### Idealised Model

The analytical model assumes:

- rigid bodies,
- frictionless joints,
- no aerodynamic drag,
- no actuator dynamics,
- no sensor noise,
- no joint backlash,
- no structural flexibility.

The Gazebo simulation may include additional numerical and rigid-body effects that are not represented in the simplified equations.

### Pendulum Inertia

The URDF pendulum is represented as a rigid body with an inertia tensor.

The simplified controller model treats the pendulum as a point mass concentrated at its centre of mass.

This difference should be recognised when comparing analytical predictions with simulation behaviour.

For a higher-fidelity controller model, the inertia term $I$ can be retained.

### Rail Limits

The equations assume that the cart can move without reaching a mechanical boundary.

The simulated rail has finite travel limits.

Therefore, the controller must also keep the cart within the available rail length.

### Upright Angle Definition

The equations use

$$
\theta=0
$$

for the upright position.

The joint angle reported by Gazebo must be converted or offset if the URDF joint coordinate does not use the same zero reference.

---

## 30. Final Result

The dynamic modelling stage produces the coupled nonlinear equations

$$
(M+m)\ddot{x}
+
ml\ddot{\theta}\cos\theta
-
ml\dot{\theta}^{2}\sin\theta
=
F
$$

and

$$
l\ddot{\theta}
+
\ddot{x}\cos\theta
-
g\sin\theta
=
0.
$$

These equations capture:

- cart inertia,
- pendulum inertia,
- gravitational effects,
- angular acceleration coupling,
- centripetal effects,
- and the influence of the control force.

They form the mathematical foundation of the inverted pendulum controller.

---

## Transition to Linearisation and State-Space Modelling

The equations derived in this chapter are nonlinear.

The LQR controller used in this project requires a linear state-space model.

The next stage therefore:

- selects the upright equilibrium point,
- applies small-angle approximations,
- linearises the nonlinear equations,
- defines the state vector,
- and constructs the state-space matrices.

Continue to:

[Dynamic Modelling: Linearisation and State-Space Representation](03_dynamic_modelling_linearization_and_state_space.md)
