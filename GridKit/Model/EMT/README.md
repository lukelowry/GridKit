# Electromagnetic Transients (EMT)

This directory contains design documentation for electromagnetic transient
(EMT) models and reusable operators in instantaneous abc coordinates.

## Conventions

These conventions reflect the current EMT model draft and may change as the
EMT design and implementation develop.

- Phase order is $a$, $b$, $c$.
- Equations use SI units unless a model states otherwise.
- Current injection terms are written as positive into buses.

## Directories

The current EMT documentation is organized into:

- `Component` (See [Component](Component/README.md))
- `Operators` (See [Operators](Operators/README.md))
- `Parameters` (See [Parameters](Parameters/README.md))

Component models are documented under `Component` because they are EMT models
connected to buses. Reusable transfer and coordinate tools are documented under
`Operators`.
Frequency-domain line-parameter models are documented under `Parameters`
because they generate sampled per-unit-length data rather than EMT bus
residuals.
