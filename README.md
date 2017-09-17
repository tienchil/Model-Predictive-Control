# Model-Predictive-Control
---

## The Model

* The model used in this project is kinematic model. See the following table for details.

State | Description
------|-------------
x     | x-position of the vehicle
y     | y-position of the vehicle
psi   | The orientation of the vehicle from positive x-direction
v     | The speed of the vehicle (mph)
CTE   | Cross Track Error
epsi  | Orientation Error

Actuators | Description
----------|-------------
delta     | Steering Angle of the vehicle {-25, +25} (degrees)
a         | Acceleration of the vehicle {-1, 1}

Parameters | Update Equations
-----------|------------------
x          | x_t+1 = x_t + v_t * cos(psi_t) * dt
y          | y_t+1 = y_t + v_t * sin(psi_t) * dt
psi        | psi_t+1 = psi_t + (v_t / Lf) * delta_t * dt
v          | v_t+1 = v_t + a_t * dt
CTE        | CTE_t+1 = f(x_t) - y_t + v_t * sin(epsi_t) * dt
epsi       | epsi_t+1 = psi_t - psi_des_t + (v_t / Lf) * delta_t * dt



## Timestep and Duration

* I have tried several combination of N (timestep length) and dt (elapsed duration between timesteps) values. N = 10 and dt = 0.1 worked the best. These values allow the vehicle to react in a reasonable time (N*dt = 1 sec).

* I tried N = 25 and dt = 0.05. However, the value of dt is too small for the vehicle to react and make a good prediction.

* N = 10 and dt = 0.2. The value of dt works for several parts of the track, but at sharp turns, the vehicle could not react fast enough to get a good predicted path.


## Polynomial and Pre-processing

* In this project, I used third degree polynomial to fit the waypoints. 

* The waypoints are tranformed into car's coordinate system to make approximations of CTE easier to calculate.

## Latency

* Latency of this project is set to be 100ms. 

* To deal with latency, before making predictions and pre-processing, I added latency terms to each received state. The following is the code.

```
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi + v*delta/2.67*latency;
v = v + acceleration*latency;
```



