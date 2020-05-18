# Writeup - Project 4, Building an Estimator
## Udacity Flying Car Nanodegree

## Overview
The project involved implementing an estimator for a quadcopter drone. The file [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) was given for reference.


## Implementation
### 01. Sensor Noise
Tasks to compute the standard deviation of measured GPS and IMU data.

Solution presented in `process_sensor_data.py` where the pandas library is  used to read in the comma separated data from `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data). The `pandas.DataFrame.std` method was then used to calculate the standard deviation with plotting used to confirm.

```py
# Load in the data
data_gps = pd.read_csv("config/log/Graph1.txt") 
data_imu = pd.read_csv("config/log/Graph2.txt") 
# Take a look at the data
fig, axes = plt.subplots(nrows=1, ncols=2)
data_gps.plot(ax=axes[0], x="time", y=" Quad.GPS.X", kind = 'line')
data_imu.plot(ax=axes[1], x="time", y=" Quad.IMU.AX", kind = 'line')
# Calculate standard deviation
sigma_gps = data_gps.std(axis=0)[" Quad.GPS.X"]
sigma_imu = data_imu.std(axis=0)[" Quad.IMU.AX"]
```

<image src="assets/Figure_1.png" height="250">  

The standard deviations were found to be:

Sensor | Standard deviation
:- | :-
GPS | 0.711
IMU | 0.489

### 02. Attitude Estimation
Task to improve the `QuadEstimatorEKF::UpdateFromIMU` method in `QuadEstimatorEKF.cpp` file.

The solution presented improves upon the small angle approximation method provided by integrating the rotation over time, dt, using the `Quaternion` class method `IntegrateBodyRate`, starting from the initial known roll, pitch and yaw angles. The Euler angles are then calculated from the quaternion and used in the complimentary filter:

<img src="https://latex.codecogs.com/gif.latex?\hat{\theta}_t&space;=&space;\frac{\tau}{\tau&space;&plus;&space;T_s}&space;\left(&space;\bar{\theta}_{t-1}&space;&plus;&space;T_s&space;z_{t,\dot{\theta}}&space;\right)&space;&plus;&space;\frac{T_s}{\tau&space;&plus;&space;T_s}&space;z_{t,\theta}" title="\hat{\theta}_t = \frac{\tau}{\tau + T_s} \left( \bar{\theta}_{t-1} + T_s z_{t,\dot{\theta}} \right) + \frac{T_s}{\tau + T_s} z_{t,\theta}" />  
<br>

<img src="https://latex.codecogs.com/gif.latex?\hat{\phi}_t&space;=&space;\frac{\tau}{\tau&space;&plus;&space;T_s}&space;\left(&space;\bar{\phi}_{t-1}&space;&plus;&space;T_s&space;z_{t,\dot{\phi}}&space;\right)&space;&plus;&space;\frac{T_s}{\tau&space;&plus;&space;T_s}&space;z_{t,\phi}" title="\hat{\phi}_t = \frac{\tau}{\tau + T_s} \left( \bar{\phi}_{t-1} + T_s z_{t,\dot{\phi}} \right) + \frac{T_s}{\tau + T_s} z_{t,\phi}" />

```cpp
Quaternion<float> q_t = Quaternion<float>().FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
V3D pqr = { gyro.x, gyro.y, gyro.z };
Quaternion<float> q_t_bar = q_t.IntegrateBodyRate(pqr, dtIMU);
float predictedRoll = q_t_bar.Roll();
float predictedPitch = q_t_bar.Pitch();
ekfState(6) = q_t_bar.Yaw();
```

### 03. Prediction Step
Task to improve the `QuadEstimatorEKF::PredictState` method in `QuadEstimatorEKF.cpp` file.

To predict the next state we use the equation:

<img src="https://latex.codecogs.com/gif.latex?g(x_t,&space;u_t,&space;\Delta&space;t)&space;=&space;\left[&space;\begin{array}{c}&space;x_{t,x}&space;&plus;&space;x_{t,\dot{x}}&space;\Delta&space;t&space;\\&space;x_{t,y}&space;&plus;&space;x_{t,\dot{y}}&space;\Delta&space;t&space;\\&space;x_{t,z}&space;&plus;&space;x_{t,\dot{z}}&space;\Delta&space;t\\&space;x_{t,\dot{x}}&space;\\&space;x_{t,\dot{y}}&space;\\&space;x_{t,\dot{z}}&space;-&space;g&space;\Delta&space;t&space;\\&space;x_{t,&space;\psi}\\&space;\end{array}\right]&space;&plus;&space;\left[&space;\begin{array}{cccc}&space;0&0&0&0\\&space;0&0&0&0\\&space;0&0&0&0\\&space;R_{bg}[0:]&&&0\\&space;R_{bg}[1:]&&&0\\&space;R_{bg}[2:]&&&0\\&space;0&0&0&1&space;\end{array}&space;\right]&space;u_t&space;\Delta&space;t" title="g(x_t, u_t, \Delta t) = \left[ \begin{array}{c} x_{t,x} + x_{t,\dot{x}} \Delta t \\ x_{t,y} + x_{t,\dot{y}} \Delta t \\ x_{t,z} + x_{t,\dot{z}} \Delta t\\ x_{t,\dot{x}} \\ x_{t,\dot{y}} \\ x_{t,\dot{z}} - g \Delta t \\ x_{t, \psi}\\ \end{array}\right] + \left[ \begin{array}{cccc} 0&0&0&0\\ 0&0&0&0\\ 0&0&0&0\\ R_{bg}[0:]&&&0\\ R_{bg}[1:]&&&0\\ R_{bg}[2:]&&&0\\ 0&0&0&1 \end{array} \right] u_t \Delta t" />

To simplify the computation we note that the second term in the expression is mainly zeros except the control input transformed by the rotation matrix:

<img src="https://latex.codecogs.com/gif.latex?\displaystyle&space;\left[\begin{matrix}0\\0\\0\\dt&space;\left(R_{00}&space;\ddot{x}^b&space;&plus;&space;R_{01}&space;\ddot{y}^b&space;&plus;&space;R_{02}&space;\ddot{z}^b\right)\\dt&space;\left(R_{10}&space;\ddot{x}^b&space;&plus;&space;R_{11}&space;\ddot{y}^b&space;&plus;&space;R_{12}&space;\ddot{z}^b\right)\\dt&space;\left(R_{20}&space;\ddot{x}^b&space;&plus;&space;R_{21}&space;\ddot{y}^b&space;&plus;&space;R_{22}&space;\ddot{z}^b\right)\\\dot{\phi}&space;dt\end{matrix}\right]" title="\displaystyle \left[\begin{matrix}0\\0\\0\\dt \left(R_{00} \ddot{x}^b + R_{01} \ddot{y}^b + R_{02} \ddot{z}^b\right)\\dt \left(R_{10} \ddot{x}^b + R_{11} \ddot{y}^b + R_{12} \ddot{z}^b\right)\\dt \left(R_{20} \ddot{x}^b + R_{21} \ddot{y}^b + R_{22} \ddot{z}^b\right)\\\dot{\phi} dt\end{matrix}\right]" />

This can be achieved by using the `Quaternion.Rotate_BtoI` method.
```cpp
VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
    assert(curState.size() == QUAD_EKF_NUM_STATES);
    Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));
    // Get acceleration in inertial frame
    V3F accel_I = attitude.Rotate_BtoI(accel);

    // Lets define predictedState = term1 + term2
    VectorXf term1 = curState;
    term1(0) += curState(3) * dt;
    term1(1) += curState(4) * dt;
    term1(2) += curState(5) * dt;
    term1(5) -= 9.81f * dt;
    VectorXf term2(7);
    term2.setZero();
    term2(3) = accel_I.x * dt;
    term2(4) = accel_I.y * dt;
    term2(5) = accel_I.z * dt;

    VectorXf predictedState = term1 + term2;    

    return predictedState;
}
```

Task to complete the `QuadEstimatorEKF::GetRbgPrime` method in `QuadEstimatorEKF.cpp` file.

This is simply completed as defined by the equation:

<img src="https://latex.codecogs.com/gif.latex?R'_{bg}&space;=&space;\left[&space;\begin{array}{ccc}&space;-\cos&space;\theta&space;\sin&space;\psi&&space;-\sin\phi&space;\sin&space;\theta&space;\sin&space;\psi&space;-&space;\cos&space;\phi&space;\cos&space;\psi&&space;-cos&space;\phi&space;\sin&space;\theta&space;\sin&space;\psi&space;&plus;&space;\sin&space;\phi&space;\cos&space;\psi\\&space;\cos&space;\theta&space;\cos&space;\psi&&space;\sin&space;\phi&space;\sin&space;\theta&space;\cos&space;\psi&space;-&space;\cos&space;\phi&space;\sin&space;\psi&&space;\cos&space;\phi&space;\sin&space;\theta&space;\cos&space;\psi&space;&plus;&space;\sin&space;\phi&space;\sin&space;\psi\\&space;0&0&0&space;\end{array}&space;\right]" title="R'_{bg} = \left[ \begin{array}{ccc} -\cos \theta \sin \psi& -\sin\phi \sin \theta \sin \psi - \cos \phi \cos \psi& -cos \phi \sin \theta \sin \psi + \sin \phi \cos \psi\\ \cos \theta \cos \psi& \sin \phi \sin \theta \cos \psi - \cos \phi \sin \psi& \cos \phi \sin \theta \cos \psi + \sin \phi \sin \psi\\ 0&0&0 \end{array} \right]" />

```cpp
MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
    MatrixXf RbgPrime(3, 3);
    RbgPrime.setZero();

    RbgPrime(0, 0) = -cosf(pitch) * sinf(yaw);
    RbgPrime(0, 1) = -sinf(roll) * sinf(pitch) * sinf(yaw) - cosf(roll) * cosf(yaw);
    RbgPrime(0, 2) = -cosf(roll) * sinf(pitch) * sinf(yaw) + sinf(roll) * cosf(yaw);
    RbgPrime(1, 0) = cosf(pitch) * cosf(yaw);
    RbgPrime(1, 1) = sinf(roll) * sinf(pitch) * cosf(yaw) - cosf(roll) * sinf(yaw);
    RbgPrime(1, 2) = cosf(roll) * sinf(pitch) * cosf(yaw) + sinf(roll) * sinf(yaw);

    return RbgPrime;
}
```

Task to complete the `QuadEstimatorEKF::Predict` method in `QuadEstimatorEKF.cpp` file.

The predict step from the EKF is completed by computing the jacobian, G_t and using in equation:

<img src="https://latex.codecogs.com/gif.latex?\bar{\Sigma}_t&space;=&space;G_t\Sigma_{t-1}G_t^T&space;&plus;&space;Q_t" title="\bar{\Sigma}_t = G_t\Sigma_{t-1}G_t^T + Q_t" />

Where Sigma is the ekfCov (covariance).

```cpp
void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
    // Predict the state forward
    VectorXf newState = PredictState(ekfState, dt, accel, gyro);

    // We'll want the partial derivative of the Rbg matrix
    MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

    // Empty Jacobian
    MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
    gPrime.setIdentity();

    // Extract rows from RbgPrime into V3F's
    V3F RbgPrime_r0 = {RbgPrime(0,0), RbgPrime(0,1), RbgPrime(0,2)};
    V3F RbgPrime_r1 = {RbgPrime(1,0), RbgPrime(1,1), RbgPrime(1,2)};
    V3F RbgPrime_r2 = {RbgPrime(2,0), RbgPrime(2,1), RbgPrime(2,2)};
    // Fill in jacobian
    gPrime(0, 3) = dt;
    gPrime(1, 4) = dt;
    gPrime(2, 5) = dt;
    gPrime(3, 6) = RbgPrime_r0.dot(accel) * dt;
    gPrime(4, 6) = RbgPrime_r1.dot(accel) * dt;
    gPrime(5, 6) = RbgPrime_r2.dot(accel) * dt;

    ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
    ekfState = newState;
}
```

### 04. Magnetometer Update
Task to complete the `QuadEstimatorEKF::UpdateFromMag` method in `QuadEstimatorEKF.cpp` file.

h_prime is found from the following equation:  

<img src="https://latex.codecogs.com/gif.latex?h'(x_t)&space;=&space;\left[\begin{array}{ccccccc}&space;0&0&0&0&0&0&1&space;\end{array}\right]" title="h'(x_t) = \left[\begin{array}{ccccccc} 0&0&0&0&0&0&1 \end{array}\right]" />

And zFromX seems to be required as the current state, normalised with respect to the measurement z although this is not stated as far as I could see. I suggest a clarification here would be useful.

```cpp
void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
    VectorXf z(1), zFromX(1);
    z(0) = magYaw;

    MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
    hPrime.setZero();

    hPrime(0, QUAD_EKF_NUM_STATES-1) = 1.0f;

    float delta = magYaw - ekfState(6);

    if (delta > F_PI)
        zFromX(0) = ekfState(6) + 2.f * F_PI;
    else if (delta < -F_PI)
        zFromX(0) = ekfState(6) - 2.f * F_PI;
    else
        zFromX(0) = ekfState(6);

    Update(z, hPrime, R_Mag, zFromX);
}
```

### 05. GPS Update
Task to complete the `QuadEstimatorEKF::UpdateFromGPS` method in `QuadEstimatorEKF.cpp` file.

The GPS measurement model is implemented with h_prime described by the following equation:

<img src="https://latex.codecogs.com/gif.latex?h'(x_t)&space;=&space;\left[\begin{array}{ccccccc}&space;1&0&0&0&0&0&0\\&space;0&1&0&0&0&0&0\\&space;0&0&1&0&0&0&0\\&space;0&0&0&1&0&0&0\\&space;0&0&0&0&1&0&0\\&space;0&0&0&0&0&1&0\\&space;\end{array}\right]" title="h'(x_t) = \left[\begin{array}{ccccccc} 1&0&0&0&0&0&0\\ 0&1&0&0&0&0&0\\ 0&0&1&0&0&0&0\\ 0&0&0&1&0&0&0\\ 0&0&0&0&1&0&0\\ 0&0&0&0&0&1&0\\ \end{array}\right]" />

The GPS standard deviations were set in `QuadEstimatorEKF.txt` using the same method as in section .01, recording additional sensor data.

```cpp
void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
    VectorXf z(6), zFromX(6);
    z(0) = pos.x;
    z(1) = pos.y;
    z(2) = pos.z;
    z(3) = vel.x;
    z(4) = vel.y;
    z(5) = vel.z;

    MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
    hPrime.setIdentity();

    // h(x_t) = x_t
    for (int i = 0; i < 6; i++) {
        zFromX(i) = ekfState(i);
    }    
    Update(z, hPrime, R_GPS, zFromX);
}
```

### 06. Adding controller
Task to add controller from project 3 and pass scenario 11.

The controller was added and passed the scenario without modification.