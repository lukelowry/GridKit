#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def sigma(x, mu):
    return 0.5 * (1.0 + np.tanh(0.5 * mu * x))


def terminal_a_phase(t, i, n, tau):
    return (np.pi / tau) * (t - 2.0 * tau * i / n)


def terminal_b_phase(t, i, n, tau):
    return (np.pi / tau) * (t - tau - 2.0 * tau * i / n)


def boundary(n):
    return np.cos(np.pi / n)


def terminal_a_coordinate(t, i, n, tau):
    return np.cos(terminal_a_phase(t, i, n, tau))


def terminal_b_coordinate(t, i, n, tau):
    return np.cos(terminal_b_phase(t, i, n, tau))


def terminal_a_selector(t, i, n, tau, mu):
    return sigma(terminal_a_coordinate(t, i, n, tau) - boundary(n), mu)


def terminal_b_selector(t, i, n, tau, mu):
    return sigma(terminal_b_coordinate(t, i, n, tau) - boundary(n), mu)


def main():
    tau = 1.0
    h = 0.25
    n = 2 * int(np.ceil(tau / h))
    mu = 5000.0

    t = np.linspace(0.0, 2.0 * tau, 4000)

    a0 = terminal_a_coordinate(t, 0, n, tau)
    b0 = terminal_b_coordinate(t, 0, n, tau)
    a0_selector = terminal_a_selector(t, 0, n, tau, mu)
    b0_selector = terminal_b_selector(t, 0, n, tau, mu)

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(t / tau, a0, label=r"terminal $a$: $\cos(\pi t/\tau)$")
    axes[0].plot(t / tau, b0, label=r"terminal $b$: $\cos(\pi(t-\tau)/\tau)$")
    axes[0].axhline(boundary(n), color="0.35", linestyle="--", linewidth=1.0, label=r"cell boundary")
    axes[0].set_ylim(-1.2, 1.2)
    axes[0].set_ylabel("coordinate")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(t / tau, a0_selector, label=r"$A_0(t)=\sigma(\cos(\pi t/\tau)-\cos(\pi/N))$")
    axes[1].plot(t / tau, b0_selector, label=r"$B_0(t)=\sigma(\cos(\pi(t-\tau)/\tau)-\cos(\pi/N))$")
    axes[1].set_ylabel("selector")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    for i in range(n):
        axes[2].plot(t / tau, terminal_a_selector(t, i, n, tau, mu))
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].set_ylabel("terminal a slots")
    axes[2].grid(True, alpha=0.25)

    for i in range(n):
        axes[3].plot(t / tau, terminal_b_selector(t, i, n, tau, mu))
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].set_ylabel("terminal b slots")
    axes[3].set_xlabel(r"$t/\tau$")
    axes[3].grid(True, alpha=0.25)

    for ax in axes[2:]:
        ax.set_yticks([0.0, 0.5, 1.0])

    fig.suptitle(rf"Two-terminal delay-ring selectors, $\mu={mu:g}$")
    fig.tight_layout()

    output = Path(__file__).with_name("delay_selector_shapes.png")
    fig.savefig(output, dpi=200)
    print(output)


if __name__ == "__main__":
    main()
