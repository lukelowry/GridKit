

This operator model is

$$
y = \delta(t-\tau) * u
$$

Notes:
- this is a reminder that $\mu$ in smoothmath needs to be configurable to the minimum time step for this to work


## CommonMath
Our common math has a pulse function (`inside`) but I want the delta distribution so we need to make this for distinction

$$
\delta(t;h) =\dfrac{1}{h} \text{bin}(t;h) = \dfrac{\sigma(t+h/2) - \sigma(t-h/2)}{h}
$$


We define the reflection indicator and incident indicator functions. These are the transport functions of the line in the coordinates of the input and output

$$
\begin{aligned}
R(t)&=\delta_{h}(1 - \cos\left(2\pi t\right)) \\
E(t)&=\delta_{h}(1 + \cos\left(2\pi t\right)) \\
\end{aligned}
$$

We then just need to apply this for each phase group (the functions for each chunk along the discretization of trasnmission line)

## Parameters

- $\tau$ the time delay
- $h$ the minimum time step resolution desired

## Model derived parameters

Determine how many internal states
$$
N=\text{ceil}\left(\dfrac{\tau}{h}\right)
$$

And 
$$
R_n(t)=R\left(\dfrac{t}{\tau}-\dfrac{n}{N}\right) 
$$

## Ports

- $u_{in}$ input signal
- $y$ output port

## Differential Variables

$$
\mathbf{x} \in \mathbb{R}^{N}
$$

## Algebraic Variables

None.


## Differential Equations

For each bus of the transmission line we need

$$
\begin{aligned}
\dot{I}_n&=\delta_a(x)\,( I^\text{inj}_a -I_n ) \\ 
\dot{I}_n&=\delta_b(x)\,( I^\text{inj}_b -I_n ) \\ 
\end{aligned}
$$

where

$$
x=1+\cos{\pi t}
$$


This approach assumes lossless, but still useful and multi modal

This approach is nice because efficient and like an integral manifold, a fast spinning tape recorder


## Algebraic Equations

None.

## Wiring

$$
y=\sum_n E\left(\dfrac{t}{\tau}-\dfrac{n}{N}\right) \, x_n
$$
