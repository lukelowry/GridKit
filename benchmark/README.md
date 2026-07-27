# GridKit benchmarks

This directory stores reproducible, local performance investigations separately
from production source and example data.

Each benchmark family has its own directory. Individual dated runs contain:

- a run report and methodology;
- raw application output and `/usr/bin/time` measurements;
- exact controlled input variants; and
- the instrumentation patch used to collect the profile.

Current benchmark families:

- [`adaptive-step/`](adaptive-step/): adaptive PhasorDynamics simulation runtime.

Benchmark outputs are evidence artifacts. They are not wired into the build or
test system and should not be committed unless explicitly requested.
