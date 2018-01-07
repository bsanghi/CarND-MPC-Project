# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program


## Project Description
The purpose of this project is to develop a nonlinear model predictive controller (NMPC) to steer a car around a track in a simulator. 
The simulator provides a feed of values containing the position of the car, its speed and heading direction. 

## The Model
The Model Predictive Controller (MPC) calculates the trajectory, actuations and sends back
 steering to the simulator. <br/>
The state vector of the vehicle is given as:
```
x - Vehicle position in forward direction
y - Vehicle position in lateral direction
psi - Angle of the vehicle (yaw angle)
v - Vehicle's speed
cte - cross-track error
epsi - orientation error

And the actuators are:
delta - Steering angle (radians)
a - acceleration

Lf - the distance between the center of mass of the vehicle and the front wheels.
```

The model is expressed by the following equations:

```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] * delta[t] / Lf * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Polynomial Fitting

The waypoints are transformed to vehicle coordinate system by translation and rotation. 
X axis aligns with the heading direction. This transformation allows to perform 
calculations consistently in vehicle coordinate system.
```
   	  // Convert from map coordinates to vehicle coordinates
          for (int i = 0; i < ptsx.size(); i++){
              auto car_coord = transfor_car_coord(psi, px, py, ptsx[i], ptsy[i]);
              ptsx_transformed[i] = car_coord[0];
              ptsy_transformed[i] = car_coord[1];
          }

          // Fit polynomial to the points - 3rd order.
          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

```
A third degree polynomial was used to compute the trajectory of the car. As mentioned in the 
lectures, most of the real world roads can be fitted with a third degree polynomial.
The functions for transformation to car coordinate and polyfit can be found in main.cpp.

## Model Predictive Control with Latency

A latency of 70ms is artificially added before sending actuations to the simulator to simulate
real world conditions.  
Use the update equations and model errors in order to factor latency in the state vector.

```
  double x_delay = 0 + ( v * cos(psi0) * delay );
  double y_delay = 0+ ( v * sin(psi0) * delay );
  double psi_delay = 0 - ( v * delta * delay / Lf );
  double v_delay = v + a * delay;
  double cte_delay = cte + ( v * sin(epsi) * delay );
  double epsi_delay = epsi + ( v * epsi * delay / Lf );
```

## Discussions & Some rubric points

First, I used 10 timesteps (`N`) of 0.1 duration (`dt`) with a ref speed of 30,50, and 70 mph.
and the following cost function used in the class.

    // The part of the cost based on the reference state.
    for( int i = 0; i < N; i++ ) {
      fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2); 
      fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); 
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
   
    for (int i = 0; i< N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2); 
    }

    for (int i = 0; i < N - 2; i++) {
      fg[0] += 600*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

### Study 1    

The above cost function worked well for the following reference speed,30 and 50 and looked ok for 70mph. 
Then, i reduced dt=0.1s to 0.07. Then, it worked well for 70mph. 

Even i wanted to increase the reference speed, the car could not turn many turns and never reach the reference 
speed because our track is relatively short and has too many turns.   

Using the following parameter for a^2 , we can increase the reference speed to 90mph and 100mph.
But, the real speed never pass 80mph because we use constant coefficient for a^2. 
The videos look ok except one turn.

for (int i = 0; i< N - 1; i++) {
   fg[0] += CppAD::pow(vars[delta_start + i], 2);
   fg[0] += (ref_v-70)*CppAD::pow(vars[a_start + i], 2);
}

### Study 2

To reach the reference speed, we have to reduce speeds when the car starts to
turn and increase the speed when the car drive in the straigth line. To do that, we have to
use dynamic coefficients instead of constant coefficients. So, I used coeffs[2] which represents
the second order term for the polynomial fit.

double a_coeff=1+coeffs[2]*coeffs[2]*1000000;

for (int i = 0; i< N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += a_coeff*CppAD::pow(vars[a_start + i], 2); 
}

when we are driving in the straight line, a_coeff gets 1. 
when we are starting turn, we will see the larger numbers
for a_coeff. I combined the above two studies which are done separately
and found the following a_coeff which contains coeff[2] and ref_v(reference speed).

double a_coeff=1+coeffs[2]*coeffs[2]*(ref_v-30)*(ref_v-30)*300;

## Results

so, I used the following numbers for final results.

N=10
dt=0.10

The final cost function :

    // The part of the cost based on the reference state.
    for( int i = 0; i < N; i++ ) {
      fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2); 
      fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); 
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    
    double a_coeff=1+coeffs[2]*coeffs[2]*(ref_v-30)*(ref_v-30)*300;

    for (int i = 0; i< N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += a_coeff*CppAD::pow(vars[a_start + i], 2); 
    }
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 600*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2); 
    }

The cost function worked well. The function is the function we used in the class except a_coeff for term a^2.

The final videos:

[ video for ref_v=50mph ](videos/final_50.mp4)  <br/>
[ video for ref_v=100mph ](videos/final_100.mp4)

The speed of car in the first video does not decrease much on turns and  is close to the reference
speed. The speed of car in the second video decreased significantly and reached around 70mph on the turns and 
increased over 90mph in the straight line. So, the algorithm worked well in the wide range of speed limits.   


