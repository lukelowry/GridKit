# Component Models

This directory contains EMT component model notes for devices connected to EMT
buses.

A component README should include:
1. Model parameters and derived parameters
2. Internal and external variables
3. Differential and algebraic equations
4. Bus residual contributions, when applicable
5. Initialization notes
6. Signal outputs, if the model has EMT signal ports

## Types

- `Branch` (See [Branch](Branch/README.md))
- `LoadRL` (See [LoadRL](LoadRL/README.md))
- `VoltageSource` (See [VoltageSource](VoltageSource/README.md))

Frequency-dependent components may reference dense rational approximation
sidecars. The sidecar schema is documented in
[../IO/FIT_JSON.md](../IO/FIT_JSON.md).
