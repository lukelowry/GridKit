# Per-case summary

Default-tolerance runs at the decade points of the mu grid.
Trajectory error is the WRMS deviation from the case's
tight-tolerance piecewise reference on the monitor grid
(`ref_model_wrms`); the piecewise row's value is that arm's own
integration-error floor. ACTIVSg200/500 and WECC240 trajectory
error is oscillation-phase dominated (see README caveat). Full
grid: results.csv; all counters: counters.md.

| case | states | horizon | arm | steps | Jacobians | Newton fails | wall (s) | traj. error |
|---|---|---|---|---|---|---|---|---|
| TGOV1 | 13 | 10 s | smooth $\mu=10^1$ | 532 | 60 | 0 | 0.01 | 4.2e-05 |
| | | | smooth $\mu=10^2$ | 511 | 57 | 0 | 0.01 | 3.2e-05 |
| | | | smooth $\mu=10^3$ | 555 | 59 | 0 | 0.01 | 2.4e-05 |
| | | | smooth $\mu=10^4$ | 601 | 71 | 7 | 0.01 | 6.9e-06 |
| | | | piecewise | 650 | 118 | 17 | 0.02 | 1.8e-06 |
| GENSAL | 34 | 30 s | smooth $\mu=10^1$ | 804 | 102 | 17 | 0.02 | 5.9e-02 |
| | | | smooth $\mu=10^2$ | 844 | 114 | 12 | 0.02 | 4.6e-03 |
| | | | smooth $\mu=10^3$ | 829 | 139 | 20 | 0.02 | 6.1e-04 |
| | | | smooth $\mu=10^4$ | 872 | 129 | 16 | 0.02 | 7.4e-05 |
| | | | piecewise | 874 | 138 | 19 | 0.02 | 3.3e-06 |
| IEEET1 | 22 | 10 s | smooth $\mu=10^1$ | 794 | 76 | 0 | 0.02 | 7.8e-03 |
| | | | smooth $\mu=10^2$ | 783 | 95 | 2 | 0.02 | 1.3e-03 |
| | | | smooth $\mu=10^3$ | 905 | 101 | 2 | 0.02 | 2.0e-04 |
| | | | smooth $\mu=10^4$ | 855 | 113 | 8 | 0.02 | 3.7e-05 |
| | | | piecewise | 945 | 174 | 26 | 0.02 | 7.6e-06 |
| GENROU control | 18 | 10 s | smooth $\mu=10^1$ | 519 | 56 | 0 | 0.02 | 4.4e-07 |
| | | | smooth $\mu=10^2$ | 519 | 56 | 0 | 0.02 | 4.4e-07 |
| | | | smooth $\mu=10^3$ | 519 | 56 | 0 | 0.02 | 4.4e-07 |
| | | | smooth $\mu=10^4$ | 519 | 56 | 0 | 0.02 | 4.4e-07 |
| | | | piecewise | 519 | 56 | 0 | 0.01 | 4.4e-07 |
| GENROU saturated | 18 | 10 s | smooth $\mu=10^1$ | 501 | 57 | 1 | 0.06 | 2.4e-02 |
| | | | smooth $\mu=10^2$ | 500 | 52 | 0 | 0.02 | 1.2e-04 |
| | | | smooth $\mu=10^3$ | 500 | 52 | 0 | 0.02 | 1.2e-04 |
| | | | smooth $\mu=10^4$ | 500 | 52 | 0 | 0.02 | 1.2e-04 |
| | | | piecewise | 500 | 52 | 0 | 0.02 | 1.2e-04 |
| GENSAL saturated | 34 | 30 s | smooth $\mu=10^1$ | 835 | 97 | 9 | 0.02 | 5.6e-02 |
| | | | smooth $\mu=10^2$ | 807 | 107 | 10 | 0.02 | 4.4e-03 |
| | | | smooth $\mu=10^3$ | 839 | 116 | 13 | 0.03 | 5.8e-04 |
| | | | smooth $\mu=10^4$ | 840 | 104 | 12 | 0.03 | 7.2e-05 |
| | | | piecewise | 777 | 143 | 23 | 0.02 | 2.8e-06 |
| TGOV1 boundary | 13 | 10 s | smooth $\mu=10^1$ | 533 | 59 | 0 | 0.06 | 7.9e-05 |
| | | | smooth $\mu=10^2$ | 541 | 59 | 0 | 0.01 | 3.2e-05 |
| | | | smooth $\mu=10^3$ | 592 | 57 | 0 | 0.02 | 2.4e-05 |
| | | | smooth $\mu=10^4$ | 605 | 69 | 8 | 0.02 | 7.0e-06 |
| | | | piecewise | 686 | 131 | 23 | 0.02 | 1.1e-06 |
| IEEET1 boundary | 22 | 10 s | smooth $\mu=10^1$ | 796 | 81 | 0 | 0.03 | 7.7e-03 |
| | | | smooth $\mu=10^2$ | 800 | 95 | 2 | 0.02 | 1.3e-03 |
| | | | smooth $\mu=10^3$ | 934 | 97 | 1 | 0.02 | 2.0e-04 |
| | | | smooth $\mu=10^4$ | 901 | 103 | 10 | 0.02 | 3.7e-05 |
| | | | piecewise | 849 | 139 | 13 | 0.02 | 4.3e-06 |
| GENSAL boundary | 34 | 30 s | smooth $\mu=10^1$ | 842 | 113 | 16 | 0.03 | 2.0e+00 |
| | | | smooth $\mu=10^2$ | 1395 | 149 | 24 | 0.03 | 1.1e+00 |
| | | | smooth $\mu=10^3$ | 4477 | 467 | 181 | 0.09 | 2.6e-01 |
| | | | smooth $\mu=10^4$ | 6307 | 728 | 311 | 0.10 | 4.0e-02 |
| | | | piecewise | 6655 | 747 | 306 | 0.11 | 1.0e-02 |
| GENROU boundary | 18 | 10 s | smooth $\mu=10^1$ | 546 | 53 | 0 | 0.02 | 3.7e-02 |
| | | | smooth $\mu=10^2$ | 522 | 58 | 0 | 0.02 | 8.4e-04 |
| | | | smooth $\mu=10^3$ | 513 | 57 | 0 | 0.02 | 1.2e-03 |
| | | | smooth $\mu=10^4$ | 513 | 57 | 0 | 0.02 | 1.2e-03 |
| | | | piecewise | 513 | 57 | 0 | 0.02 | 1.2e-03 |
| ACTIVSg200 | 882 | 10 s | smooth $\mu=10^1$ | 681 | 50 | 0 | 0.06 | 6.1e-01 |
| | | | smooth $\mu=10^2$ | 724 | 76 | 8 | 0.06 | 1.4e-01 |
| | | | smooth $\mu=10^3$ | 968 | 102 | 12 | 0.07 | 1.2e-01 |
| | | | smooth $\mu=10^4$ | 692 | 123 | 14 | 0.07 | 4.3e-02 |
| | | | piecewise | 1094 | 155 | 27 | 0.08 | 1.0e-01 |
| ACTIVSg500 | 1722 | 10 s | smooth $\mu=10^1$ | 355 | 51 | 0 | 0.08 | 1.5e-01 |
| | | | smooth $\mu=10^2$ | 505 | 63 | 5 | 0.08 | 1.7e-01 |
| | | | smooth $\mu=10^3$ | 513 | 85 | 13 | 0.08 | 1.7e-01 |
| | | | smooth $\mu=10^4$ | 510 | 88 | 15 | 0.08 | 1.7e-01 |
| | | | piecewise | 518 | 100 | 19 | 0.08 | 1.7e-01 |
| WECC240 | 4292 | 10 s | smooth $\mu=10^1$ | 990 | 53 | 0 | 0.26 | 1.3e+01 |
| | | | smooth $\mu=10^2$ | 754 | 58 | 0 | 0.22 | 1.5e-01 |
| | | | smooth $\mu=10^3$ | 921 | 114 | 8 | 0.26 | 1.4e-02 |
| | | | smooth $\mu=10^4$ | 950 | 160 | 5 | 0.30 | 1.2e-02 |
| | | | piecewise | 1017 | 218 | 20 | 0.28 | 6.6e-03 |
| ACTIVSg10k | 73998 | 10 s | smooth $\mu=10^1$ | 1211 | 46 | 0 | 8.60 | -- |
| | | | smooth $\mu=10^2$ | 1304 | 67 | 10 | 11.24 | -- |
| | | | smooth $\mu=10^3$ | 1609 | 164 | 39 | 12.54 | -- |
| | | | smooth $\mu=10^4$ | 1728 | 362 | 89 | 16.73 | -- |
| | | | piecewise | 558 | 423 | 134 | 10.05 | -- **fails** |
