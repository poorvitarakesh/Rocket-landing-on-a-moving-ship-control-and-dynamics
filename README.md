# Robust Fuel-Optimal and Predictive Control of a Rocket Landing on a Moving Ship

MATLAB simulation of a 2D rocket landing on an oscillating ship. Run one file: `rocket_landing_single.m`.

## Method
- Planner: fuel-optimal trajectory (direct transcription, trapezoidal collocation, fmincon SQP)
- Estimator: model prediction + measurement correction; also estimates wind and thrust error
- Controllers compared: open-loop, optimal + PD, optimal + nonlinear MPC (4 s horizon)
- Monte Carlo study: random wind, thrust bias, sensor noise

## Requirements
- MATLAB R2016b or later (R2019b+ for yline)
- Optimization Toolbox

## How to run
1. Open `rocket_landing_single.m` in MATLAB and press Run.
2. Switches are at the bottom of `params()`: `p.animate`, `p.saveVideo`, `p.runMC`, `p.nMC`.

## Results (single run, wind + noise + 5% thrust shortfall)
| Case | t [s] | posErr [m] | relVx [m/s] | vy [m/s] | Fuel [kg] | Success |
|---|---|---|---|---|---|---|
| Open-loop, nominal | 24.95 | 0.00 | -0.12 | -0.62 | 184.6 | Yes |
| Open-loop, disturbed | 16.50 | 78.65 | -10.28 | -25.61 | 116.5 | No |
| Optimal + PD | 24.24 | 1.15 | -2.22 | -2.45 | 179.0 | No |
| Optimal + MPC | 26.61 | 0.05 | -0.02 | -0.23 | 194.8 | Yes |

## Monte Carlo (10 runs)
| Controller | Success % | Mean abs err [m] | Max abs err [m] | Mean fuel [kg] |
|---|---|---|---|---|
| Open-loop | 0 | 80.74 | 103.20 | 137.7 |
| Optimal + PD | 20 | 1.28 | 3.89 | 183.1 |
| Optimal + MPC | 100 | 0.24 | 0.73 | 194.2 |

Open-loop fuel is lower only because those runs crash early.

![Results](results/fig1_results.png)
![Estimator](results/fig2_estimator.png)
![Monte Carlo](results/fig4_montecarlo.png)

## Limitations
2D model, no attitude dynamics, known ship motion, no minimum throttle, only 10 Monte Carlo runs.

## Future work
Convex optimization (SOCP) comparison, EKF/UKF, robust MPC, real-time solver (CasADi/acados).
