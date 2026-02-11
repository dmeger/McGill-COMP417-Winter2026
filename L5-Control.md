# Robotic Control

Distinct from planning, robot control indicates the problem of determining instantaneous control actions from instantaneously observed states $u_t = \pi(x_t)$. We may or may not deliberate about a long horizon behavior in order to come up with $\pi$, but when it comes to applying the function to run the robot, it is meant to be a "pause and deliberate" but rather an "act now, act fast" sort of outcome. However, as we'll see, computing the form, weights, code or structure of $\pi$ could still take plenty of deliberation, only it should happen in advance, or between episodes, etc.

## Example Control Problems

1) The block on ice is a unit mass object sliding without friction on the x line. Its control goal is to be at rest at $x=0$, but it is a real physical object and moves with some momentum, having acceleration coupled to the force applied, $\ddot{x} = u$, $\dot{x} = \dot{x}(0) + ut$ and $x = x(0) + \dot{x}(0)t + ut^2$ (by integration in time). 

2) The friction-less pendulum system is one which only feels forces from gravity around its single attachment point. Leverage traslates the gravitational force into a torque, $-mlg\sin(\theta)$, for mass $m$ and length, $l$. Balancing this with the 2nd form definition of torque, change in angular momentum, $\frac{dL}{dt}$, and writing out the angular momentum, $L=r \times p = ml^2\frac{d\theta}{dt}$ lets us take the derivative $\frac{dL}{dt} = ml^2\frac{d^2\theta}{dt^2}$. We can equate the two formulas for torque: $-mlg\sin(\theta) = ml^2\frac{d^2\theta}{dt^2}$ and find the overall motion equations $\frac{d^2\theta}{dt^2} + \frac{g}{l}\sin(\theta) = 0$. We might be familiar with the motion of a pendulum swinging slowly near it's downward point. When we have a "small angle", we can assume $\sin(\theta) = \theta$, which simplifies our equations to $\frac{d^2\theta}{dt^2} + \frac{g}{l}\theta = 0$. These are now equivalent to a mass oscillating on a simple spring, with a sinusoidal state trajectory in time. How about the full non-linear pendulum's time trajectories? They are not easy to write out in general, as we can see by the behavior of the phase-space diagram:

<img src="https://en.wikipedia.org/wiki/Pendulum_(mechanics)#/media/File:Pendulum_phase_portrait.svg">

## Taking Control of the Phase Space

The fact that a physical dynamic system moves in time following the rules of its motion equations (visualized by the phase space), is kind of inconvenient when working with them, but we must embrace it and come up with ways for our robot's controllers, $u=\pi(x)$ to move the system as we desire. A first strategy can be direct analysis (by hand) of the phase space.

In example one, our block on ice can be made more interesting by limiting the controls possible to the range $|u|<1$. We can either accelerate to the left $u=-1$, passively drift $u=0$ or accelerate to the right $u=+1$, and each of these motions will 