# Dynamic Modelling: Linearization and State-Space Representation

## Purpose

The equations of motion derived in the previous chapter accurately describe the nonlinear dynamics of the inverted pendulum system.

However, nonlinear equations cannot be directly used with most modern linear control techniques such as Linear Quadratic Regulator (LQR).

Therefore, before designing the controller, the nonlinear dynamic model must be transformed into an equivalent linear model around a selected operating point.

This chapter explains:

- why linearisation is required,
- how the nonlinear equations are linearised,
- how the operating point is selected,
- how small-angle approximations are applied,
- and how the resulting linear equations are prepared for state-space representation.

The state-space model obtained in this chapter forms the mathematical foundation of the LQR controller implemented later in this project.

---

# Linearization Workflow

The linearization process used in this project follows the sequence below.

```text
Nonlinear Equations of Motion
        ↓
Select Operating Point
        ↓
Apply Taylor Series Expansion
        ↓
Apply Small-Angle Approximations
        ↓
Obtain Linear Differential Equations
        ↓
Prepare for State-Space Representation
```

---

# 1. Why Linearization Is Required

The equations derived using Newton–Euler and Lagrangian mechanics are nonlinear.

For this project, the nonlinear equations are

$$
(M+m)\ddot{x}+ml\ddot{\theta}\cos\theta-ml\dot{\theta}^2\sin\theta=F
$$

$$
l\ddot{\theta}+\ddot{x}\cos\theta-g\sin\theta=0
$$

These equations contain nonlinear functions such as

$$
\sin\theta
$$

$$
\cos\theta
$$

and

$$
\dot{\theta}^2\sin\theta
$$

Because of these nonlinear terms:

- the principle of superposition does not hold,
- the system matrices cannot be written as constant matrices,
- the dynamics vary continuously with the pendulum angle,
- and standard linear control techniques cannot be applied directly.

Although nonlinear controllers can be designed for these equations, the objective of this project is to stabilise the pendulum around its upright equilibrium using an LQR controller.

Since LQR requires a linear state-space model, the nonlinear equations must first be linearised.

---

# 2. Local Linear Approximation

Linearization does not replace the nonlinear model.

Instead, it creates a local approximation that accurately represents the system behaviour only near a chosen operating point.

Graphically, the nonlinear curve is replaced by its tangent line around the equilibrium.

Near this operating point, both models produce nearly identical behaviour.

As the system moves farther away from the equilibrium, the approximation becomes less accurate.

For this reason, the LQR controller designed in this project is intended only for stabilising the pendulum after it is already close to the upright position.

Large-angle swing-up control requires the original nonlinear model.

---

# 3. Selecting the Operating Point

Linearization must always be performed around a specific operating point.

An operating point is a system condition where all state variables remain constant if no disturbance occurs.

For the inverted pendulum, two equilibrium positions exist.

## Stable Equilibrium

The pendulum hanging downward

$$
\theta=\pi
$$

or

$$
\theta=-\pi
$$

This configuration is naturally stable because gravity restores the pendulum after a small disturbance.

---

## Unstable Equilibrium

The pendulum balanced upright

$$
\theta=0
$$

This configuration is naturally unstable because gravity causes the pendulum to fall after even a very small disturbance.

Since the objective of this project is to balance the pendulum upright, the operating point is selected as

$$
x=0
$$

$$
\dot{x}=0
$$

$$
\theta=0
$$

$$
\dot{\theta}=0
$$

This operating point represents the desired equilibrium around which the controller will regulate the system.

---

# 4. Taylor Series Expansion

The mathematical basis of linearization is the Taylor series expansion.

For a general nonlinear function

$$
f(x)
$$

the Taylor series about an operating point

$$
x=x_0
$$

is

$$
f(x)=f(x_0)+\frac{df}{dx}\Big|_{x_0}(x-x_0)+\frac{1}{2!}\frac{d^2f}{dx^2}\Big|_{x_0}(x-x_0)^2+\cdots
$$

The first term represents the value of the function at the operating point.

The second term represents the local slope.

The remaining terms represent higher-order nonlinear effects.

For controller design, only the first-order approximation is retained.

Therefore,

$$
f(x)\approx f(x_0)+\frac{df}{dx}\Big|_{x_0}(x-x_0)
$$

This approximation converts nonlinear functions into linear expressions that are valid near the selected operating point.

---

# 5. Small-Angle Approximation

Since the controller operates only near the upright equilibrium,

$$
\theta\approx0
$$

the pendulum angle remains very small during normal operation.

Therefore, several standard approximations can be applied.

## Approximation of Sine

The Taylor expansion of the sine function is

$$
\sin\theta=\theta-\frac{\theta^3}{3!}+\frac{\theta^5}{5!}-\cdots
$$

When

$$
|\theta|\ll1
$$

all higher-order terms become negligible.

Therefore,

$$
\sin\theta\approx\theta
$$

---

## Approximation of Cosine

The Taylor expansion of the cosine function is

$$
\cos\theta=1-\frac{\theta^2}{2!}+\frac{\theta^4}{4!}-\cdots
$$

Near the upright equilibrium,

$$
\cos\theta\approx1
$$

---

## Approximation of the Centripetal Term

The nonlinear equations also contain

$$
\dot{\theta}^2\sin\theta
$$

Using

$$
\sin\theta\approx\theta
$$

this term becomes

$$
\dot{\theta}^2\theta
$$

This expression is the product of three small quantities.

Since it is a higher-order nonlinear term, it is neglected during linearisation.

Therefore,

$$
\dot{\theta}^2\sin\theta\approx0
$$

---

# 6. Summary of the Small-Angle Approximations

The following approximations are used throughout this project.

| Nonlinear Expression | Linear Approximation |
|----------------------|----------------------|
| $$\sin\theta$$ | $$\theta$$ |
| $$\cos\theta$$ | $$1$$ |
| $$\dot{\theta}^2\sin\theta$$ | $$0$$ |

These approximations are valid only when the pendulum remains close to the upright equilibrium.

If the pendulum angle becomes large, the nonlinear equations must be used instead.

---

# 7. Linearization of the First Equation of Motion

The original nonlinear cart equation is

$$
(M+m)\ddot{x}+ml\ddot{\theta}\cos\theta-ml\dot{\theta}^2\sin\theta=F
$$

Applying the small-angle approximations

$$
\cos\theta\approx1
$$

and

$$
\dot{\theta}^2\sin\theta\approx0
$$

gives

$$
(M+m)\ddot{x}+ml\ddot{\theta}=F
$$

This is the linearised horizontal equation of motion.

The nonlinear centripetal term has disappeared because it is negligible near the equilibrium.

---

# 8. Linearization of the Second Equation of Motion

The original nonlinear pendulum equation is

$$
l\ddot{\theta}+\ddot{x}\cos\theta-g\sin\theta=0
$$

Applying

$$
\cos\theta\approx1
$$

and

$$
\sin\theta\approx\theta
$$

gives

$$
l\ddot{\theta}+\ddot{x}-g\theta=0
$$

This is the linearised rotational equation.

The gravitational term is now proportional to the pendulum angle, which makes the system linear.

---

# 9. Final Linear Differential Equations

After linearisation, the nonlinear model becomes

$$
(M+m)\ddot{x}+ml\ddot{\theta}=F
$$

$$
l\ddot{\theta}+\ddot{x}-g\theta=0
$$

These equations describe the system only near the upright equilibrium.

Unlike the original nonlinear equations, they contain no trigonometric functions and no higher-order products of state variables.

This makes them suitable for conversion into state-space form.

---

# 10. How Linearization Is Used in This Project

The analytical model developed in the previous chapter represents the complete nonlinear dynamics of the inverted pendulum.

However, the controller implemented in this project is not designed directly from those nonlinear equations.

Instead, the nonlinear model is transformed into the linear differential equations derived above.

These linear equations provide the starting point for constructing the state-space representation used by the LQR controller.

The next stage converts the second-order differential equations into a first-order state-space model by defining the system state variables and deriving the system matrices.

Continue to:

**Dynamic Modelling: State-Space Representation**