# Breaker Model

`Breaker` is scaffolded for a two-port three-phase EMT breaker. The full model
spec was not present as a README in `lukel/emt-system-dev`, so this file records
the preserved public shape only.

## Parameters

| Parameter | Description |
| --- | --- |
| `closed` | Phase mask for initially closed phases. Defaults to `abc`. |

## Ports

| Port | Description |
| --- | --- |
| `from` | First EMT bus connection |
| `to` | Second EMT bus connection |

## Variables

The scaffold reserves three algebraic current variables, one per phase.

Equation implementation is intentionally stubbed in this branch.
