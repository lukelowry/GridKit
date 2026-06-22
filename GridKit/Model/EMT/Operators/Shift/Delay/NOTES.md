

This operator model is

$$
y = \delta(t-\tau) * u
$$


## CommonMath
Our common math has a pulse function (`inside`) but I want the delta distribution so we need to make this for distinction

$$
\delta_T(t) =\dfrac{1}{T} \text{bin}(t;T) = \dfrac{\sigma(2t+T) - \sigma(2t-T)}{T}
$$


We define the reflection indicator and incident indicator functions:

$$
\begin{aligned}
R(t)&=\delta_{h}(1 - \cos\left(\pi t\right)) \\
E(t)&=\delta_{h}(\cos\left(\pi t\right))
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

## Differential Variables

$$
\mathbf{x} \in \mathbb{R}^{N}
$$

## Algebraic Variables

None.


## Differential Equations



$$
\dot{x}_n=R\left(t-\tau\dfrac{n}{N}\right) \, u_\text{in}
$$

## Algebraic Equations

None.

## Wiring

$$
y=\sum_n E\left(t-\tau\dfrac{n}{N}\right) \, x_n
$$
