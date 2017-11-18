# Compilation

- The code successfully compiles without errors using '''cmake''' and '''make'''

# Implementation

## The Model

- The model used for the Model Predictive Control was based on the classroom modules.
    - Define the length of the trajectory, N, and duration of each timestep, dt.
    - Define actuators and update equations along with other constraints.
    - Define the cost function. (based on CTE, velocity and orientation - different weights are used unique to each factor)
    - Pass current state as initial state, use Ipopt optimization solver to return vector of control inputs that minimizes the cost function, apply control input and repeat this step

#### State
- The state vector can be represented as [x,y,ψ,v].  
  - x = x position
  - y = y position
  - ψ = vehicle orientation
  - v = velocity

#### Actuators
- We use two actuators that control this system [δ, a]
  - δ = Steering angle
  - a = Acceleration (-ve for braking and +ve for acceleration)
  
#### Update Equations
- The update equations used to predict the next step is based on the following equations
```
x' = x + v*cos(psi)*dt
y' = y + v*sin(psi)*dt
psi' = psi + v/Lf*δ*dt
v' = v + a*dt
```
## Timestep Length and Elapsed Duration (N and dt)

#### Timestep Length N (N=10 for this model)
- This is the number of timesteps the model uses for future state predictions based on our current state. The higher the number, the further ahead the model predicts. Inherently, this also increases computation time. Conversely, a lower N means the number of steps into the future that it predicts is low.

#### Elapsed Duration dt (dt=0.1 for this model)
- This is the length of each timestep between actuations. 
- A large value results in a larger gap between actuations which makes the model less accurate. Lowering that value makes the model more accurate at a computational cost. A balance between the two needs to be chosen to ensure the model performs well.
- An optimal value of Time T = N * dt is chosen to ensure the prediction isn't too much into the future as the track conditions can change rapidly as the vehicle speed increases rendering the prediction useless.

## Polynomial Fitting and MPC Pre-Processing

- First waypoints are transformed from simulator's global co-ordinates to car co-ordinates (Main.cpp lines 107-112) by subtracting out px and py and applying 2D rotation.
- polyfit() is used to fit a 3rd degree polynomial line to the transformed points.
- Since we are using vehicle co-ordinates, px, py and psi are all 0.
- CTE is calculated using polyeval() at px(=0)

## Model Predictive Control with Latency

- Latency is added to the system to correct for delays in the model reacting to the predictions. This can either be a turn being actuated later than it should or that it stops turning a little too late which will both cause the car to veer off the track. As one would expect, the effect of this is increased with speed
- A latency of 100ms was implemented. ie., the model predicts the state 100ms ahead of time and feeds it to the MPC solver
- Code used for this is implemented in Main.cpp (Lines 100 - 104)
'''
double latency = 0.1;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi - v*steer_value/Lf*latency;
v = v + throttle_value*latency;
'''

# Simulation

- The car successfully drives around the track using the Model Predictive Control 
