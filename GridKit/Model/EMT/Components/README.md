# EMT Components

EMT components use instantaneous `abc` phase variables and a single current-injection convention:

```text
A port injection row holds current flowing into the connected node from the component.
Every bus equation is sum(injections into the node) = 0.
```

Consequences for the initial component set:

```text
VoltageSource:
  INJ = (E_inst - V) / r

LoadRL:
  EQ  = R*i + L*i' + v = 0
  INJ = +i
  The load current variable is oriented from the load into the bus.

BranchLumpedConstant:
  i flows from -> to
  IF = -G_half*v_f - C_half*v_f' - i
  IT = -G_half*v_t - C_half*v_t' + i
  EQ = R*i + L*i' + v_t - v_f = 0
```

Do not reinterpret these signs independently inside a component. Tests must include value-level
checks for this convention in addition to finite-difference Jacobian checks.
