# Physical Modelling of the Inverted Pendulum

## 1. Introduction

Before deriving equations of motion or designing a controller, the real physical system must first be converted into a clear and consistent engineering model.

This process is called **physical modelling**.

Physical modelling does not begin with equations. It begins by identifying:

- what the real system contains,
- how its parts are connected,
- which motions are possible,
- which forces and moments act on the system,
- which physical effects are important,
- which effects may be neglected,
- which coordinates describe the motion,
- and how the mathematical variables correspond to the simulated or physical components.

For the inverted pendulum project, the objective of physical modelling is to represent the cart–pendulum mechanism in a form suitable for:

1. free-body analysis,
2. dynamic modelling,
3. linearization,
4. state-space representation,
5. LQR controller design,
6. and ROS 2–Gazebo implementation.

This document explains the physical modelling process from the general engineering perspective and then applies each step to the inverted pendulum system used in this project.

---

## 2. What Is a Physical Model?

A **physical model** is a simplified representation of a real system that preserves the physical properties required for a particular analysis.

A real mechanism may contain:

- flexible materials,
- bearing friction,
- manufacturing tolerances,
- joint backlash,
- sensor noise,
- actuator dynamics,
- structural vibration,
- air resistance,
- electrical delays,
- and many other effects.

Including every physical detail would make the model unnecessarily complex. Therefore, an engineering model is created by retaining the effects that are important for the current objective and neglecting effects that have a small influence.

For example, the physical model required for mechanical design may be different from the model required for controller design.

A structural analysis may require:

- stress,
- deformation,
- material properties,
- and detailed geometry.

A controller design model may instead require:

- mass,
- inertia,
- degrees of freedom,
- applied forces,
- joint positions,
- joint velocities,
- and equilibrium points.

The physical model used in this project is therefore a **control-oriented mechanical model**.

Its purpose is not to reproduce every detail of a real inverted pendulum. Its purpose is to represent the dominant mechanical behaviour required to derive the system dynamics and design a balancing controller.

---

## 3. From a Real System to an Engineering Model

A general physical modelling workflow can be written as:

```text
Real physical system
        ↓
Identify components and connections
        ↓
Determine degrees of freedom
        ↓
Define coordinates and sign conventions
        ↓
Identify forces and moments
        ↓
Define physical parameters
        ↓
State modelling assumptions
        ↓
Construct free-body diagrams
        ↓
Prepare the system for dynamic modelling
```

Each step must be completed consistently. An error made during physical modelling will propagate into the equations of motion, state-space model, controller design, and software implementation.

Typical physical modelling errors include:

- selecting an incorrect rotation axis,
- using inconsistent positive directions,
- measuring the pendulum angle from the wrong reference,
- confusing total pendulum length with centre-of-mass distance,
- omitting an important force,
- applying a force to the wrong body,
- using an incorrect inertia,
- or mixing simulation coordinates with mathematical coordinates.

---

## 4. Physical Description of the Inverted Pendulum

The system considered in this project consists of four main elements:

1. a fixed world,
2. a horizontal rail,
3. a cart that moves along the rail,
4. and a pendulum connected to the cart by a revolute joint.

The rail is fixed relative to the world. The cart can translate along the rail, while the pendulum can rotate about the joint located on the cart.

The control input is a horizontal force applied to the cart.

The purpose of the controller is to generate a force that moves the cart in such a way that the pendulum remains near its upright equilibrium position.

A simplified representation is:

```text
                         Pendulum
                            │
                            │
                            ●  Centre of mass
                            │
                            │
                            O  Revolute joint / pivot
                     ┌────────────┐
                     │    Cart    │  → F
                     └────────────┘
════════════════════════════════════════════  Rail
```

The pendulum is naturally unstable in the upright position. A small angular deviation causes gravity to move it farther away from equilibrium unless the cart is actively accelerated to restore balance.

---

## 5. System Components

### 5.1 World

The world represents the fixed inertial reference frame.

All positions, velocities, forces, and orientations are ultimately interpreted relative to this frame.

In the project, the world is represented by the Gazebo world and the fixed `world` link in the robot description.

---

### 5.2 Rail

The rail constrains the cart to move along one horizontal direction.

The rail itself does not move because it is fixed to the world.

Its functions are:

- defining the permitted cart motion,
- limiting the cart travel,
- and providing the mechanical reference for the prismatic joint.

In the project, the rail is represented by `rail_link`.

---

### 5.3 Cart

The cart is the translating body on which the control force acts.

Its horizontal position changes according to:

- the applied control force,
- the reaction produced by the pendulum,
- its mass,
- and any additional physical effects such as friction.

In the mathematical model, the cart mass is represented by:

$$
M
$$

In the project, the cart is represented by `cart_link`.

---

### 5.4 Pendulum

The pendulum is a rigid body connected to the cart by a revolute joint.

Its motion is rotational, but the pendulum centre of mass also undergoes translational motion because the pivot moves with the cart.

This coupling between cart translation and pendulum rotation is the central physical property of the inverted pendulum system.

In the mathematical model, the pendulum mass is represented by:

$$
m
$$

The total physical pendulum length is represented by:

$$
L
$$

The distance between the pivot and the pendulum centre of mass is represented by:

$$
l
$$

For a uniform rod whose pivot is located at one end:

$$
l = \frac{L}{2}
$$

In this project:

$$
L = 0.5\ \text{m}
$$

and, under the uniform-rod assumption:

$$
l = 0.25\ \text{m}
$$

The distinction between \(L\) and \(l\) is important. The equations of motion usually use the pivot-to-centre-of-mass distance \(l\), not necessarily the full geometric length \(L\).

In the project, the pendulum is represented by `pendulum_link`.

---

### 5.5 Pivot

The pivot is the rotational connection between the cart and the pendulum.

It allows the pendulum to rotate while forcing the upper end of the pendulum to move together with the cart.

The pivot may exert reaction forces on the pendulum in both horizontal and vertical directions.

These forces are internal to the complete cart–pendulum system. They may appear when the cart and pendulum are analysed separately, but they cancel when the complete system is considered.

In the project, the pivot is represented by the revolute joint:

```text
pendulum_cart_joint
```

---

## 6. Degrees of Freedom

A **degree of freedom** is an independent coordinate required to describe the configuration of a mechanical system.

The inverted pendulum has two independent degrees of freedom:

1. horizontal cart translation,
2. pendulum rotation.

The cart translation is described by:

$$
x
$$

The pendulum rotation is described by:

$$
\theta
$$

Therefore, the configuration of the system can be written as:

$$
q = \begin{bmatrix} x \\
\theta \end{bmatrix}
$$

where \(q\) is the vector of generalized coordinates.

The system has two degrees of freedom even though many points in the mechanism have both horizontal and vertical coordinates. Those point coordinates are not independent. Once \(x\) and \(\theta\) are known, the positions of all relevant points can be calculated.

---

## 7. Generalized Coordinates

A **generalized coordinate** is an independent variable used to describe the configuration of a mechanical system.

Generalized coordinates do not have to be Cartesian coordinates. They may be:

- linear positions,
- rotation angles,
- joint displacements,
- or any independent variables that fully define the system configuration.

For the inverted pendulum:

$$
q_1 = x
$$

$$
q_2 = \theta
$$

The generalized-coordinate vector is therefore:

$$ 
q = \begin{bmatrix} q_1 \\
q_2 \end{bmatrix} = \begin{bmatrix} x \\
\theta \end{bmatrix}
$$

Their first derivatives are:

$$ 
\dot{q} = \begin{bmatrix} \dot{x} \\
\dot{\theta} \end{bmatrix}
$$

and their second derivatives are:

$$ 
\ddot{q} = \begin{bmatrix} \ddot{x} \\
\ddot{\theta} \end{bmatrix}
$$

where:

- \(x\) is the cart position,
- \(\dot{x}\) is the cart velocity,
- \(\ddot{x}\) is the cart acceleration,
- \(\theta\) is the pendulum angle,
- \(\dot{\theta}\) is the pendulum angular velocity,
- \(\ddot{\theta}\) is the pendulum angular acceleration.

---

## 8. Coordinate System

A coordinate system must be defined before forces, velocities, and angles are assigned signs.

For this project, the following convention is used:

- the \(x\)-axis is horizontal,
- positive \(x\) points along the positive cart motion direction,
- the vertical axis points upward,
- the cart moves only along the \(x\)-axis,
- the pendulum rotates in the plane perpendicular to its revolute-joint axis.

A simplified coordinate representation is:

```text
                  +vertical
                      ↑
                      │
                      │
                      O────────────→ +x
                    Pivot
```

The exact Gazebo axis depends on the URDF joint definitions, but the mathematical model is reduced to a two-dimensional plane containing:

- the cart translation direction,
- and the pendulum rotation.

The mathematical sign convention must remain consistent with:

- the URDF joint axis,
- the measured joint position,
- the measured joint velocity,
- and the direction of the applied force.

If the simulation reports an angle with the opposite sign, the controller software must apply the appropriate sign conversion.

---

## 9. Pendulum Angle Definition

The pendulum angle must be defined relative to a specific reference direction.

In this project, the controller model uses the upright position as the equilibrium:

$$
\theta = 0
$$

Therefore:

- \(\theta = 0\) represents the upright pendulum,
- a positive or negative value represents angular displacement from the upright position,
- and the controller attempts to drive \(\theta\) back toward zero.

This convention is particularly suitable for linearization and LQR design because the controller is designed around the upright equilibrium.

A conceptual representation is:

```text
                    θ = 0
                      │
                      │
                      O
                 ┌─────────┐
                 │  Cart   │
                 └─────────┘
```

When the pendulum rotates away from the upright position:

```text
                       /
                      /  θ
                     /
                    O
               ┌─────────┐
               │  Cart   │
               └─────────┘
```

The direction defined as positive must be used consistently throughout all equations.

### 9.1 Mathematical Angle and Joint Angle

The mathematical angle and the angle reported by the simulation are not automatically guaranteed to be identical.

The Gazebo joint position depends on:

- the joint axis,
- the parent and child link frames,
- the URDF joint origin,
- and the initial link orientation.

Therefore, the software may require a transformation such as:

$$ 
\theta_{\text{model}} = s_\theta \left( \theta_{\text{joint}}-\theta_{\text{offset}} \right)
$$

where:

- \(\theta_{\text{joint}}\) is the measured joint angle,
- \(\theta_{\text{offset}}\) aligns the joint reference with the upright position,
- \(s_\theta\) is either \(+1\) or \(-1\), depending on the joint direction.

The correct mapping must be verified by moving the pendulum in the simulator and observing the sign of the reported joint angle.

---

## 10. Position of the Pendulum Centre of Mass

The pendulum centre of mass position is required for both Newton–Euler and Lagrange modelling.

Let:

- \(x\) be the cart position,
- \(l\) be the distance from the pivot to the pendulum centre of mass,
- \(\theta\) be the pendulum angle measured from the upright vertical direction.

Using the chosen coordinate convention, the centre-of-mass coordinates can be written as:

$$
x_p = x + l\sin\theta
$$

$$
y_p = l\cos\theta
$$

where:

- \(x_p\) is the horizontal position of the pendulum centre of mass,
- \(y_p\) is the vertical position of the pendulum centre of mass.

These equations show that the centre of mass moves because of two effects:

1. translation of the cart through \(x\),
2. rotation of the pendulum through \(\theta\).

The horizontal centre-of-mass position contains both effects:

$$
x_p = x + l\sin\theta
$$

The vertical centre-of-mass position depends only on the pendulum angle:

$$
y_p = l\cos\theta
$$

At the upright position:

$$
\theta = 0
$$

therefore:

$$
x_p = x
$$

$$
y_p = l
$$

This means the centre of mass is directly above the pivot by the distance \(l\).

---

## 11. Velocity of the Pendulum Centre of Mass

The centre-of-mass velocity is obtained by differentiating the position equations with respect to time.

Starting with:

$$
x_p = x + l\sin\theta
$$

differentiate with respect to time:

$$
\dot{x}_p = \dot{x} + l\dot{\theta}\cos\theta
$$

Similarly:

$$
y_p = l\cos\theta
$$

therefore:

$$
\dot{y}_p = -l\dot{\theta}\sin\theta
$$

These expressions are required when calculating the pendulum kinetic energy.

The squared centre-of-mass speed is:

$$
v_p^2 = \dot{x}_p^2+\dot{y}_p^2
$$

Substituting the velocity components:

$$
v_p^2 = \left( \dot{x} + l\dot{\theta}\cos\theta \right)^2 + \left( -l\dot{\theta}\sin\theta \right)^2
$$

After expansion:

$$
v_p^2 = \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2
$$

because:

$$
\sin^2\theta+\cos^2\theta=1
$$

This result later appears in the kinetic-energy expression used in the Lagrange method.

---

## 12. Acceleration of the Pendulum Centre of Mass

The centre-of-mass acceleration is obtained by differentiating the velocity equations.

From:

$$
\dot{x}_p = \dot{x} + l\dot{\theta}\cos\theta
$$

the horizontal acceleration becomes:

$$
\ddot{x}_p = \ddot{x} + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta
$$

The three terms represent:

- \(\ddot{x}\): acceleration caused by cart translation,
- \(l\ddot{\theta}\cos\theta\): horizontal component of tangential acceleration,
- \(-l\dot{\theta}^2\sin\theta\): horizontal component of centripetal acceleration.

From:

$$
\dot{y}_p = -l\dot{\theta}\sin\theta
$$

the vertical acceleration becomes:

$$
\ddot{y}_p = -l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta
$$

The centre-of-mass acceleration expressions are required when applying Newton's second law directly to the pendulum.

---

## 13. Forces Acting on the System

The main forces in the idealized model are:

1. the control force applied to the cart,
2. the weight of the pendulum,
3. the weight of the cart,
4. the rail reaction force,
5. the pivot reaction forces.

Depending on the modelling detail, friction may also be included.

---

### 13.1 Control Force

The controller generates a horizontal force:

$$
F
$$

This force acts on the cart along the rail direction.

The sign convention is:

- \(F>0\): force in the positive \(x\)-direction,
- \(F<0\): force in the negative \(x\)-direction.

In the project, this force is sent to the Gazebo joint-force interface associated with the cart prismatic joint.

---

### 13.2 Pendulum Weight

The gravitational force acting on the pendulum is:

$$
W_p = mg
$$

and acts vertically downward through the pendulum centre of mass.

In vector form:

$$
\mathbf{W}_p = \begin{bmatrix} 0 \\
-mg \end{bmatrix}
$$

Gravity is responsible for the instability of the upright equilibrium.

When the pendulum deviates from the upright position, gravity produces a moment about the pivot that tends to increase the angular deviation.

---

### 13.3 Cart Weight

The gravitational force acting on the cart is:

$$
W_c = Mg
$$

The cart cannot move vertically because the rail constrains it. Therefore, the cart weight is balanced by the vertical reaction force from the rail.

For the horizontal dynamic equation, the cart weight does not appear directly because it has no horizontal component.

---

### 13.4 Rail Reaction

The rail applies a constraint force to the cart.

This reaction:

- supports the cart and pendulum vertically,
- prevents motion perpendicular to the rail,
- and allows the cart to move only along the permitted axis.

The rail reaction is not usually required in the final horizontal equations because the constrained vertical motion of the cart is already known.

---

### 13.5 Pivot Reaction Forces

When the cart and pendulum are analysed separately, the pivot exerts reaction forces.

They may be written as:

$$
H
$$

for the horizontal reaction and:

$$
V
$$

for the vertical reaction.

The forces acting on the pendulum at the pivot are equal and opposite to the forces acting on the cart:

$$
\mathbf{F}_{\text{cart on pendulum}} = - \mathbf{F}_{\text{pendulum on cart}}
$$

These are internal forces for the complete cart–pendulum system.

In a Newton–Euler derivation, the reaction forces may first appear in separate body equations and then be eliminated.

In a Lagrange derivation, ideal constraint forces are usually not written explicitly.

---

## 14. Free-Body Diagrams

A **free-body diagram**, or FBD, is a diagram showing a selected body isolated from its surroundings together with all external forces and moments acting on it.

The purpose of an FBD is to convert the physical mechanism into a form suitable for Newton's and Euler's equations.

A correct FBD must:

- isolate one body or a clearly defined group of bodies,
- remove the physical connections,
- replace each removed connection with the corresponding reaction forces or moments,
- include gravity,
- include applied forces,
- show relevant dimensions,
- define positive coordinate directions,
- and remain consistent with the chosen sign convention.

---

## 15. General Procedure for Constructing an FBD

A systematic FBD procedure is:

### Step 1: Select the Body

Decide whether the analysis concerns:

- the cart,
- the pendulum,
- or the complete cart–pendulum system.

### Step 2: Isolate the Body

Conceptually remove all surrounding components.

### Step 3: Replace Connections with Reactions

For example:

- replace the pivot with horizontal and vertical reaction forces,
- replace the rail contact with a support reaction,
- retain the externally applied cart force.

### Step 4: Add Gravity

Apply each body's weight at its centre of mass.

### Step 5: Define Coordinates

Show:

- positive translation direction,
- positive rotation direction,
- relevant angles.

### Step 6: Add Geometric Quantities

Include:

- pendulum length,
- centre-of-mass distance,
- and position variables.

### Step 7: Verify Completeness

Check whether every external interaction has been represented.

---

## 16. Cart Free-Body Diagram

The cart is affected by:

- the applied control force \(F\),
- the horizontal pivot reaction,
- the cart weight \(Mg\),
- and the rail reaction.

A conceptual cart FBD is:

```text
                     ↑ N
                     │
              ┌────────────┐
        ← H   │    Cart    │   → F
              └────────────┘
                     │
                     ↓ Mg
```

where:

- \(N\) is the vertical rail reaction,
- \(Mg\) is the cart weight,
- \(H\) is the horizontal interaction force from the pendulum,
- \(F\) is the applied control force.

Since the cart has no vertical acceleration:

$$
\sum F_y = 0
$$

The horizontal equation is related to:

$$
\sum F_x = M\ddot{x}
$$

The exact sign of \(H\) depends on the chosen direction for the reaction force.

---

## 17. Pendulum Free-Body Diagram

The pendulum is affected by:

- horizontal pivot reaction \(H\),
- vertical pivot reaction \(V\),
- gravitational force \(mg\).

A conceptual pendulum FBD is:

```text
                         ●  Centre of mass
                         ↓ mg
                        /
                       /
                      /
                     O
                  → H
                  ↑ V
```

The translational equations of the pendulum centre of mass are:

$$
\sum F_x = m\ddot{x}_p
$$

$$
\sum F_y = m\ddot{y}_p
$$

The rotational equation about the centre of mass or pivot may be written using:

$$
\sum \tau = I\alpha
$$

where:

- \(I\) is the relevant moment of inertia,
- \(\alpha = \ddot{\theta}\) is the angular acceleration.

---

## 18. Complete-System Free-Body Diagram

The cart and pendulum may also be considered as a single system.

In that case, the pivot reaction forces are internal and do not appear in the complete-system FBD.

The principal external horizontal force is:

$$
F
$$

The total horizontal momentum depends on both:

- the cart translation,
- and the horizontal motion of the pendulum centre of mass.

This complete-system view is useful because it helps eliminate the internal pivot force.

---

## 19. External and Internal Forces

The distinction between external and internal forces is essential.

### External Forces

External forces are applied by objects outside the selected system.

For the complete cart–pendulum system, examples include:

- control force \(F\),
- gravity,
- rail reaction,
- friction, if included.

### Internal Forces

Internal forces act between components inside the selected system.

For the complete cart–pendulum system, the pivot reaction forces are internal.

Internal forces do not disappear physically. However, when all bodies are considered together, equal and opposite internal force pairs cancel in the total force balance.

---

## 20. Moments and Torque

A force produces a moment about a point when its line of action does not pass through that point.

The moment is:

$$
\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}
$$

For planar motion, the scalar moment can be written as:

$$
\tau_z = r_xF_y-r_yF_x
$$

where:

- \(\mathbf{r}\) is the position vector from the selected moment point to the force application point,
- \(\mathbf{F}\) is the applied force.

For the pendulum, gravity acts through the centre of mass and produces a moment about the pivot.

The magnitude of this gravitational moment is proportional to:

$$
mgl\sin\theta
$$

The sign depends on the angular convention.

For the inverted configuration, the gravitational moment drives the pendulum away from the upright equilibrium.

---

## 21. Mass Moment of Inertia

The pendulum does not behave as a point mass unless that approximation is deliberately selected.

A rigid body's resistance to angular acceleration is described by its mass moment of inertia.

For a uniform slender rod of total length \(L\), the moment of inertia about its centre of mass is:

$$
I_{\text{COM}} = \frac{1}{12}mL^2
$$

Using the parallel-axis theorem, the moment of inertia about one end is:

$$
I_{\text{pivot}} = I_{\text{COM}} + m\left(\frac{L}{2}\right)^2
$$

Therefore:

$$
I_{\text{pivot}} = \frac{1}{3}mL^2
$$

If the pendulum is approximated as a point mass located at distance \(l\), then:

$$
I_{\text{pivot}} = ml^2
$$

These models are not identical.

The project must use a convention consistent with:

- the URDF inertial parameters,
- the mathematical equations,
- and the controller model.

Because the Gazebo pendulum is represented as a rigid cylinder, its inertia is determined by the cylinder geometry and the inertial properties defined in the URDF/Xacro model.

The control-oriented equations used in this project employ a simplified pendulum representation. Any difference between the simplified analytical inertia and the detailed simulated inertia contributes to model mismatch.

---

## 22. Model Parameters

The principal physical parameters are:

| Symbol | Description | Unit |
|---------|-------------|------|
| $M$ | Cart mass | kg |
| $m$ | Pendulum mass | kg |
| $L$ | Total pendulum length | m |
| $l$ | Pivot-to-centre-of-mass distance | m |
| $I$ | Pendulum mass moment of inertia | kg·m² |
| $g$ | Gravitational acceleration | m/s² |
| $F$ | Horizontal control force | N |
| $x$ | Cart position | m |
| $\theta$ | Pendulum angular displacement | rad |

For this project, the nominal physical parameters are listed below.

| Symbol | Description | Project Value |
|---------|-------------|---------------|
| $M$ | Cart mass | 3.0 kg |
| $m$ | Pendulum mass | 1.0 kg |
| $L$ | Total pendulum length | 0.5 m |
| $l$ | Pivot-to-centre-of-mass distance | 0.25 m (assuming a uniform rod) |
| $g$ | Gravitational acceleration | 9.81 m/s² |
| - | Rail length | 1.0 m |
| - | Cart travel limit | Approximately −0.5 m to +0.5 m |

The values listed above correspond to the physical parameters defined in the URDF/Xacro model used throughout this project.

> **Note:** The value of **$l$** (pivot-to-centre-of-mass distance) must match the location of the pendulum's inertial origin specified in the URDF/Xacro model. If the analytical model and the URDF use different centre-of-mass locations, the simulated dynamics will differ from the mathematical model, resulting in model mismatch.

---

## 23. Modelling Assumptions

An engineering model must state its assumptions explicitly.

The assumptions used for the analytical inverted-pendulum model are listed below.

### 23.1 Planar Motion

The mechanism is assumed to move in a single vertical plane.

Therefore:

- the cart has one translational degree of freedom,
- the pendulum has one rotational degree of freedom,
- lateral motion is neglected.

---

### 23.2 Rigid Bodies

The cart, rail, and pendulum are assumed to be rigid.

Therefore:

- elastic deformation is neglected,
- structural vibration is neglected,
- link geometry does not change during motion.

---

### 23.3 Ideal Joints

The prismatic and revolute joints are initially treated as ideal.

This means:

- no joint backlash,
- no joint clearance,
- no compliance,
- no unmodelled joint deformation.

---

### 23.4 Frictionless Motion

The initial analytical model neglects:

- cart–rail friction,
- pivot friction,
- viscous damping,
- rolling resistance.

This simplifies the equations and isolates the dominant nonlinear behaviour.

Gazebo may still contain numerical damping or joint properties depending on the simulation configuration. These create model mismatch between the analytical and simulated systems.

---

### 23.5 Uniform Gravitational Field

Gravity is assumed constant:

$$
g = 9.81\ \text{m/s}^2
$$

and acts vertically downward.

---

### 23.6 Fixed Rail

The rail is assumed perfectly fixed relative to the world.

Therefore, motion of the complete support structure is not included in the analytical model.

---

### 23.7 Horizontal Rail

The rail is assumed perfectly horizontal.

A rail inclination would introduce an additional component of gravity along the cart direction.

---

### 23.8 Direct Force Input

The control input is assumed to be a direct horizontal force acting on the cart:

$$
u = F
$$

The analytical model does not initially include:

- motor electrical dynamics,
- gearbox dynamics,
- actuator current limits,
- force transmission dynamics,
- or actuator delay.

---

### 23.9 Known Parameters

Masses, length, centre-of-mass position, inertia, and gravity are assumed to be known.

In a real system, these values may contain uncertainty.

---

### 23.10 Upright Equilibrium

The controller-oriented model is developed around the upright equilibrium:

$$
\theta = 0
$$

This is the unstable equilibrium point that the LQR controller attempts to stabilize.

---

## 24. Why Assumptions Are Necessary

Assumptions are not arbitrary omissions. They define the scope of the model.

A useful model must balance:

- physical accuracy,
- mathematical complexity,
- computational cost,
- and suitability for the intended engineering task.

For example, including motor inductance and bearing friction may improve realism, but these effects are not required to understand the basic cart–pendulum coupling or derive an initial LQR controller.

The correct approach is usually:

1. begin with the simplest model that captures the dominant behaviour,
2. design and test the controller,
3. compare predicted and observed behaviour,
4. add neglected effects only when they materially affect performance.

---

## 25. Relationship Between the Analytical Model and the Gazebo Model

The analytical model and the Gazebo model serve different purposes.

### Analytical Model

The analytical model is used to:

- understand the physics,
- derive equations of motion,
- linearize the dynamics,
- construct the state-space model,
- and calculate the LQR gain.

### Gazebo Model

The Gazebo model is used to:

- simulate rigid-body motion,
- apply gravity,
- enforce joint constraints,
- calculate collisions,
- integrate the system dynamics numerically,
- provide joint-state measurements,
- and receive control-force commands.

The controller does not send the equations of motion to Gazebo.

Instead:

1. the equations are derived analytically,
2. the equations are used to design the LQR controller,
3. the controller calculates a force from measured states,
4. the force is sent to Gazebo,
5. Gazebo calculates the resulting motion using its own physics engine.

This distinction is essential:

> The analytical model is used for controller design, while Gazebo independently simulates the physical response of the robot model.

---

## 26. Mapping the Mathematical Model to the ROS 2–Gazebo Model

The mathematical variables correspond to the project components as follows:

| Mathematical Quantity | Physical Meaning | Project Representation |
|-----------------------|------------------|------------------------|
| $M$ | Cart mass | Mass of `cart_link` |
| $m$ | Pendulum mass | Mass of `pendulum_link` |
| $L$ | Total pendulum length | Length of `pendulum_link` |
| $l$ | Pivot-to-centre-of-mass distance | Distance from the joint origin to the inertial origin of `pendulum_link` |
| $I$ | Pendulum mass moment of inertia | Inertia tensor of `pendulum_link` defined in the URDF/Xacro model |
| $x$ | Cart position | Position of `cart_rail_joint` |
| $\dot{x}$ | Cart velocity | Velocity of `cart_rail_joint` |
| $\theta$ | Pendulum angular displacement | Position of `pendulum_cart_joint` |
| $\dot{\theta}$ | Pendulum angular velocity | Velocity of `pendulum_cart_joint` |
| $F$ | Horizontal control force | Force command applied to `cart_rail_joint` |
| - | Rail constraint | Prismatic motion of `cart_rail_joint` along the x-axis |
| - | Pivot constraint | Revolute motion of `pendulum_cart_joint` about the y-axis |

---

## 27. URDF/Xacro Representation

The robot description contains the physical structure used by ROS 2 and Gazebo.

The main elements are:

```text
world
  └── world_rail_joint
       └── rail_link
            └── cart_rail_joint
                 └── cart_link
                      └── pendulum_cart_joint
                           └── pendulum_link
```

### 27.1 Fixed Joint

`world_rail_joint` fixes the rail to the world.

This implements the analytical assumption that the support structure does not move.

### 27.2 Prismatic Joint

`cart_rail_joint` allows translation along the rail.

It represents the generalized coordinate:

$$
x
$$

Its position and velocity provide:

$$
x \quad\text{and}\quad \dot{x}
$$

### 27.3 Revolute Joint

`pendulum_cart_joint` allows pendulum rotation.

It represents the generalized coordinate:

$$
\theta
$$

Its position and velocity provide the raw joint measurements used to determine:

$$
\theta \quad\text{and}\quad \dot{\theta}
$$

### 27.4 Inertial Properties

Each movable link requires:

- mass,
- centre-of-mass location,
- and inertia tensor.

Gazebo uses these values to calculate the dynamic response.

The analytical model must use equivalent or intentionally simplified values.

---

## 28. Physical Consistency Checks

Before deriving equations or tuning a controller, the following consistency checks should be performed.

### 28.1 Joint Axis Check

Move the cart and pendulum manually in the simulator and verify:

- the cart translates along the intended axis,
- the pendulum rotates in the intended plane,
- no unintended motion is permitted.

### 28.2 Sign Check

Apply a positive cart force and verify:

- whether cart position increases,
- whether the reported cart velocity is positive.

Move the pendulum in the mathematically positive direction and verify:

- whether the reported joint angle increases,
- whether the angular velocity sign matches the mathematical convention.

### 28.3 Upright-Zero Check

Place the pendulum in the upright position and verify whether the joint position is:

$$
0\ \text{rad}
$$

If not, determine the required angular offset.

### 28.4 Centre-of-Mass Check

Verify that the pendulum inertial origin is located at the intended centre of mass.

For a uniform rod:

$$
l = \frac{L}{2}
$$

### 28.5 Inertia Check

Verify that:

- inertia values are non-zero,
- units are \(\text{kg}\cdot\text{m}^2\),
- the inertia tensor corresponds to the link geometry and orientation.

### 28.6 Mass Check

Verify that the masses in the analytical model match the URDF/Xacro masses.

### 28.7 Limit Check

Verify that the cart travel limits in the model correspond to the physical rail length.

### 28.8 Gravity Check

Verify that gravity is enabled and has the expected direction and magnitude.

---

## 29. Dimensional Consistency

Every physical equation must be dimensionally consistent.

For example:

$$
F = ma
$$

has units:

$$
\text{N} = \text{kg}\cdot\frac{\text{m}}{\text{s}^2}
$$

The term:

$$
ml\ddot{\theta}
$$

has units:

$$
\text{kg}\cdot\text{m}\cdot\frac{1}{\text{s}^2} = \text{N}
$$

because angular acceleration in radians per second squared is dimensionally treated as \(1/\text{s}^2\).

The term:

$$
mgl
$$

has units:

$$
\text{kg}\cdot\frac{\text{m}}{\text{s}^2}\cdot\text{m} = \text{N}\cdot\text{m}
$$

which is torque.

Dimensional analysis is a powerful method for detecting modelling mistakes.

---

## 30. Common Physical Modelling Mistakes

### 30.1 Confusing \(L\) and \(l\)

The full pendulum length and the centre-of-mass distance are not always the same.

For a uniform rod:

$$
l=\frac{L}{2}
$$

### 30.2 Using an Inconsistent Angle Reference

An angle measured from the downward vertical cannot be inserted directly into equations derived for an angle measured from the upward vertical.

### 30.3 Reversing the Joint Sign

The URDF joint axis may define positive rotation opposite to the mathematical convention.

### 30.4 Omitting Pendulum Translational Motion

The pendulum centre of mass moves because the pivot translates with the cart. Treating the pendulum as rotating about a fixed world point would be incorrect.

### 30.5 Ignoring the Pendulum Inertia Definition

A point-mass pendulum and a rigid rod pendulum have different inertia properties.

### 30.6 Mixing Internal and External Forces

Pivot forces are internal for the complete system but external when the cart or pendulum is isolated.

### 30.7 Applying the Control Force to the Wrong Coordinate

The input force acts on the cart translation, not directly on the pendulum angle.

### 30.8 Ignoring Joint and Travel Limits

The theoretical model may allow unlimited cart motion, while the simulated and real mechanisms have finite rail length.

---

## 31. Final Physical Model

After applying the modelling assumptions and definitions, the inverted pendulum is represented as:

- a cart of mass \(M\),
- constrained to horizontal translation \(x\),
- a pendulum of mass \(m\),
- with pivot-to-centre-of-mass distance \(l\),
- connected to the cart by an ideal revolute joint,
- subject to gravitational acceleration \(g\),
- controlled by a horizontal force \(F\),
- and described by generalized coordinates \(x\) and \(\theta\).

The generalized-coordinate vector is:

$$
q = \begin{bmatrix} x \\
\theta \end{bmatrix}
$$

The centre-of-mass position is:

$$
x_p = x+l\sin\theta
$$

$$
y_p = l\cos\theta
$$

The control input is:

$$
u=F
$$

The upright equilibrium is:

$$
\theta=0
$$

This physical model provides the foundation for deriving the nonlinear equations of motion.

---

## 32. Transition to Dynamic Modelling

Physical modelling answers the questions:

- What bodies exist?
- How are they connected?
- What motions are possible?
- Which coordinates describe the system?
- Which forces act?
- Which parameters are required?
- Which assumptions are used?

Dynamic modelling answers the next question:

> How do the applied forces, masses, geometry, and motion variables determine the accelerations of the system?

The following document develops the nonlinear equations of motion using:

1. the Newton–Euler method,
2. and the Lagrange method.

See:

[Dynamic Modelling: Equations of Motion](02_dynamic_modelling_equations_of_motion.md)
