# martian-mindset-mech-control-test

# Mechanics & Control Test — Martian Mindset  
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

The repository accompanies the PDF report and contains:

* **Jupyter notebooks** that walk through Exercises 4 and 5 step-by-step.  
* **MATLAB scripts** for pendulum energy-pump / swing-up simulations.    
* **LaTeX source** of the report (with live hyperlinks back to the code).  

---

## Repository layout

```
martian-mindset-mech-control-test/
├─ code/
│   ├─ control/               
│   │   ├─ pendulum/               # MATLAB scripts  ⇦ main pendulum work happens here
│   │   │   ├─ simulate_pendulum.m, render_pendulum.m, euler_integrator.m, etc
│   │   └─ exercise_8.m
│   └─ notebooks/             # Jupyter notebooks (Exercises 1 – 8)
├─ docs/                      # LaTeX + compiled PDF
├─ figs/                      # Auto-generated figures used in the report
├─ .vscode/                   # Optional editor tasks & lint settings
├─ requirements.txt           # Minimal Python deps (numpy, matplotlib, sympy)
├─ README.md                  # ← you are here
└─ LICENSE
```
## git clone https://github.com/Awai005/martian-mindset-mech-control-test.git
cd martian-mindset-mech-control-test

## Run the MATLAB pendulum simulation
Open MATLAB / Octave in code/control/ and simply execute:

"simulate_pendulum.m"

```matlab
% simulate_pendulum.m
%
% Mode selection:    'free_fall' or 'swing_up'
params.mode = 'swing_up';

% Integrator choice: 'euler' or 'rk4'
integrator  = 'rk4';

% Controller (swing_up only): 'pd' or 'lqr'
params.controller = 'lqr';
```
