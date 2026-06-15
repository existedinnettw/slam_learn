# EKF-SLAM

> [!NOTE]
> Following document is not yet proofreading. Error may existed.

[EKF SLAM - Robot Mapping WS 2017/18](http://ais.informatik.uni-freiburg.de/teaching/ws17/mapping/pdf/slam05-ekf-slam.pdf)

EKF-SLAM applies the extended Kalman filter to the joint problem of estimating
the robot pose and a map of landmarks.

For online SLAM, the desired belief is

$$
p(x_t,m\mid z_{1:t},u_{1:t}),
$$

where

* $x_t$ is the current robot pose;
* $m$ is the map;
* $u_{1:t}$ are the controls or odometry measurements;
* $z_{1:t}$ are the sensor observations.

EKF-SLAM approximates this belief by one joint Gaussian:

$$
p(x_t,m\mid z_{1:t},u_{1:t})
\approx
\mathcal N(\mu_t,\Sigma_t).
$$

The important change from an ordinary localization EKF is the meaning of the
state:

| Filter | Estimated state | Map |
| --- | --- | --- |
| EKF localization | Robot pose $x_t$ | Known and fixed |
| EKF-SLAM | Robot pose $x_t$ and landmarks $m$ | Unknown and estimated |

In localization, a landmark observation corrects only the robot pose because
the landmark position is treated as known. In EKF-SLAM, the same observation
corrects both the robot pose and the map because both are uncertain.

This chapter assumes:

* a planar robot pose $(x,y,\theta)$;
* static 2D point landmarks;
* known correspondence between a measurement and its landmark during the
  basic EKF correction;
* Gaussian motion and observation noise;
* nonlinear models approximated by first-order linearization.

Data association and new-landmark initialization are discussed separately
because they are not part of the ordinary EKF equations.

## State vector

For a robot moving in a 2D plane, the robot pose is

$$
x_t=
\begin{bmatrix}
x_t^r\\
y_t^r\\
\theta_t
\end{bmatrix}.
$$

Each point landmark has a 2D position:

$$
m_j=
\begin{bmatrix}
m_{j,x}\\
m_{j,y}
\end{bmatrix}.
$$

With $N$ landmarks, the complete state is

$$
\mu_t=
\begin{bmatrix}
x_t^r\\
y_t^r\\
\theta_t\\
m_{1,x}\\
m_{1,y}\\
\vdots\\
m_{N,x}\\
m_{N,y}
\end{bmatrix}
\in\mathbb R^{2N+3}.
$$

Although the compact notation

$$
\mu_t=[x_t,m_1,\ldots,m_N]^T
$$

looks like it has only $N+1$ entries, $x_t$ contains three scalar values and
every $m_j$ contains two. Therefore, the state has $3+2N$ scalar dimensions.

For example, with three landmarks,

$$
\mu_t=
\begin{bmatrix}
x_t^r&
y_t^r&
\theta_t&
m_{1,x}&m_{1,y}&
m_{2,x}&m_{2,y}&
m_{3,x}&m_{3,y}
\end{bmatrix}^T
\in\mathbb R^9.
$$

The notation contains four conceptual objects, one robot pose and three
landmarks, but those objects contain nine scalar state variables.

> [!NOTE]
>
> The dimension depends on the chosen map representation. A 3D point, line,
> plane, or other landmark type requires a different number of parameters.

## Joint covariance

The covariance has size

$$
\Sigma_t\in\mathbb R^{(2N+3)\times(2N+3)}.
$$

It can be viewed as a block matrix:

$$
\Sigma_t=
\begin{bmatrix}
\Sigma_{xx} & \Sigma_{xm}\\
\Sigma_{mx} & \Sigma_{mm}
\end{bmatrix}.
$$

Here,

* $\Sigma_{xx}$ is uncertainty in the robot pose;
* $\Sigma_{mm}$ is uncertainty in the landmarks;
* $\Sigma_{xm}$ and $\Sigma_{mx}$ are correlations between the robot and map.

The correlations are essential. When the robot observes one known landmark,
the EKF correction can improve the estimated robot pose and, through the joint
covariance, change the estimates of other landmarks.

Writing every landmark block explicitly gives

$$
\Sigma_t=
\begin{bmatrix}
\Sigma_{xx} &
\Sigma_{xm_1} &
\cdots &
\Sigma_{xm_N}\\
\Sigma_{m_1x} &
\Sigma_{m_1m_1} &
\cdots &
\Sigma_{m_1m_N}\\
\vdots &
\vdots &
\ddots &
\vdots\\
\Sigma_{m_Nx} &
\Sigma_{m_Nm_1} &
\cdots &
\Sigma_{m_Nm_N}
\end{bmatrix}.
$$

The block dimensions are:

* $\Sigma_{xx}\in\mathbb R^{3\times3}$;
* $\Sigma_{xm_j}\in\mathbb R^{3\times2}$;
* $\Sigma_{m_im_j}\in\mathbb R^{2\times2}$.

A covariance block is not a geometric landmark. It describes how estimation
errors vary together. For example, a nonzero $\Sigma_{m_1m_2}$ means that an
error in the estimated position of landmark $1$ provides information about
the likely error in landmark $2$.

### Why landmarks become correlated

Suppose the robot observes landmark $1$ from an uncertain pose. The inferred
position of landmark $1$ depends on that pose, so their errors become
correlated. Later, the robot moves and observes landmark $2$. Landmark $2$ is
also initialized relative to the uncertain robot pose. Both landmarks now
share uncertainty through the robot trajectory.

Conceptually:

```text
landmark 1 <- observation <- robot pose -> observation -> landmark 2
```

This shared uncertainty is why observing landmark $1$ later can indirectly
improve the estimate of landmark $2$. It is also why setting each landmark's
covariance independently would lose important SLAM information.

As more landmarks are added, the covariance becomes large and dense. A direct
EKF-SLAM correction has roughly quadratic cost in the number of landmarks,
which limits classic EKF-SLAM to relatively small maps.

## From EKF to EKF-SLAM

The ordinary EKF uses a nonlinear state transition

$$
x_t=g(u_t,x_{t-1})+\epsilon_t
$$

and nonlinear observation

$$
z_t=h(x_t)+\delta_t.
$$

EKF-SLAM uses exactly the same filtering equations. The difference is that the
state is augmented to include the map:

$$
y_t=
\begin{bmatrix}
x_t\\m
\end{bmatrix}.
$$

To avoid confusing the complete SLAM state with the measurement residual,
this chapter writes the complete state mean as $\mu_t$ and later writes the
innovation as $\nu_t$.

The augmented nonlinear transition is

$$
g^{\text{SLAM}}(u_t,y_{t-1})
=
\begin{bmatrix}
g(u_t,x_{t-1})\\
m
\end{bmatrix}.
$$

The landmark part is copied unchanged because the world is assumed static.
The augmented observation for landmark $j$ is

$$
h_j^{\text{SLAM}}(y_t)=h(x_t,m_j).
$$

Thus:

$$
\text{EKF-SLAM}
=
\text{ordinary EKF applied to one joint robot-map state}.
$$

## Motion prediction

Assume the robot motion model is

$$
x_t=g(u_t,x_{t-1})+\epsilon_t,
\qquad
\epsilon_t\sim\mathcal N(0,R_t).
$$

Static landmarks do not move:

$$
m_{j,t}=m_{j,t-1}.
$$

Therefore, the prediction changes the robot portion of the mean but leaves the
landmark means unchanged:

$$
\bar\mu_t=
\begin{bmatrix}
g(u_t,\mu_{x,t-1})\\
\mu_{m,t-1}
\end{bmatrix}.
$$

The augmented motion Jacobian has the form

$$
G_t=
\begin{bmatrix}
G_{x,t} & 0\\
0 & I
\end{bmatrix},
$$

where $G_{x,t}$ is the Jacobian of the robot motion model. Process noise is
added only to the moving robot state:

$$
R_t^{\text{aug}}=
\begin{bmatrix}
R_{x,t} & 0\\
0 & 0
\end{bmatrix}.
$$

The predicted covariance is

$$
\bar\Sigma_t
=
G_t\Sigma_{t-1}G_t^T+R_t^{\text{aug}}.
$$

The map is not independently predicted because the landmarks are assumed
static. However, their uncertainty remains correlated with the predicted
robot pose.

### Deriving the augmented motion Jacobian

The joint transition is

$$
g^{\text{SLAM}}(u_t,y_{t-1})
=
\begin{bmatrix}
g(u_t,x_{t-1})\\
m_{t-1}
\end{bmatrix}.
$$

Differentiate it with respect to the complete previous state:

$$
G_t
=
\frac{\partial g^{\text{SLAM}}}{\partial y}
=
\begin{bmatrix}
\frac{\partial g}{\partial x} &
\frac{\partial g}{\partial m}\\
\frac{\partial m}{\partial x} &
\frac{\partial m}{\partial m}
\end{bmatrix}.
$$

The robot motion model does not depend on landmark positions, so

$$
\frac{\partial g}{\partial m}=0.
$$

Static landmark positions do not depend on the robot pose, and copying the map
has derivative identity:

$$
\frac{\partial m}{\partial x}=0,
\qquad
\frac{\partial m}{\partial m}=I.
$$

Therefore,

$$
G_t=
\begin{bmatrix}
G_{x,t} & 0\\
0 & I
\end{bmatrix}.
$$

### Block form of covariance prediction

Substitute the block matrices into

$$
\bar\Sigma_t=G_t\Sigma_{t-1}G_t^T+R_t^{\text{aug}}.
$$

The result is

$$
\bar\Sigma_t=
\begin{bmatrix}
G_{x,t}\Sigma_{xx}G_{x,t}^T+R_{x,t} &
G_{x,t}\Sigma_{xm}\\
\Sigma_{mx}G_{x,t}^T &
\Sigma_{mm}
\end{bmatrix}.
$$

This equation shows precisely what prediction does:

* robot uncertainty changes and receives new motion noise;
* robot-map cross-covariance is transformed by the robot motion Jacobian;
* map covariance remains unchanged because static landmarks do not move.

The landmark means and $\Sigma_{mm}$ staying unchanged does **not** mean that
the map has become known. It means that motion alone supplies no new
information about the static map.

### Example motion model

For a simple velocity model with control

$$
u_t=
\begin{bmatrix}
v_t\\\omega_t
\end{bmatrix},
$$

and time interval $\Delta t$, one possible motion model is

$$
g(u_t,x_{t-1})=
\begin{bmatrix}
x_{t-1}^r+v_t\Delta t\cos\theta_{t-1}\\
y_{t-1}^r+v_t\Delta t\sin\theta_{t-1}\\
\theta_{t-1}+\omega_t\Delta t
\end{bmatrix}.
$$

Its Jacobian with respect to the robot pose is

$$
G_{x,t}=
\begin{bmatrix}
1 & 0 & -v_t\Delta t\sin\theta_{t-1}\\
0 & 1 & v_t\Delta t\cos\theta_{t-1}\\
0 & 0 & 1
\end{bmatrix}.
$$

Only this $3\times3$ robot Jacobian must be derived from the motion model. It
is then embedded into the larger $(2N+3)\times(2N+3)$ SLAM Jacobian.

## Landmark observation

Suppose the robot observes landmark $j$ using a range-bearing sensor. Define

$$
\Delta x_j=m_{j,x}-x_t^r,
\qquad
\Delta y_j=m_{j,y}-y_t^r,
\qquad
q_j=\Delta x_j^2+\Delta y_j^2.
$$

The expected observation is

$$
h_j(\mu_t)=
\begin{bmatrix}
\sqrt{q_j}\\
\operatorname{atan2}(\Delta y_j,\Delta x_j)-\theta_t
\end{bmatrix}.
$$

The real measurement is modeled as

$$
z_t^j=h_j(\mu_t)+\delta_t,
\qquad
\delta_t\sim\mathcal N(0,Q_t).
$$

The sensor does not directly observe the global coordinates $(m_{j,x},m_{j,y})$.
It observes the landmark relative to the robot. Both the robot pose and
landmark position are therefore inputs to the observation function.

At the predicted mean, define

$$
\hat z_t^j=h_j(\bar\mu_t).
$$

This is what the sensor is expected to measure if the predicted robot and
landmark estimates are correct.

### Linearizing the landmark observation

As in the ordinary EKF, linearize the nonlinear observation around the
predicted mean:

$$
h_j(y_t)
\approx
h_j(\bar\mu_t)
+
H_t^j(y_t-\bar\mu_t).
$$

The Jacobian with respect to the complete SLAM state is

$$
H_t^j
=
\left.
\frac{\partial h_j(y)}{\partial y}
\right|_{y=\bar\mu_t}.
$$

The observation depends only on the robot pose and landmark $j$. Consequently,
the observation Jacobian is mostly zero:

$$
H_t^j=
\begin{bmatrix}
H_x^j & 0 & \cdots & H_{m_j}^j & \cdots & 0
\end{bmatrix}.
$$

For a system containing $N$ landmarks,

$$
H_t^j\in\mathbb R^{2\times(2N+3)}.
$$

It has two rows because a range-bearing observation contains two scalar
measurements. It has $2N+3$ columns because the observation is linearized with
respect to every scalar in the joint state.

For the range-bearing model,

$$
H_x^j=
\begin{bmatrix}
-\frac{\Delta x_j}{\sqrt{q_j}} &
-\frac{\Delta y_j}{\sqrt{q_j}} &
0\\
\frac{\Delta y_j}{q_j} &
-\frac{\Delta x_j}{q_j} &
-1
\end{bmatrix}
$$

and

$$
H_{m_j}^j=
\begin{bmatrix}
\frac{\Delta x_j}{\sqrt{q_j}} &
\frac{\Delta y_j}{\sqrt{q_j}}\\
-\frac{\Delta y_j}{q_j} &
\frac{\Delta x_j}{q_j}
\end{bmatrix}.
$$

The zero blocks say that changing an unrelated landmark while holding the
robot and observed landmark fixed does not directly change the predicted
measurement.

For example, with three landmarks and an observation of landmark $2$,

$$
H_t^2=
\begin{bmatrix}
H_x^2 & 0_{2\times2} & H_{m_2}^2 & 0_{2\times2}
\end{bmatrix}.
$$

Although this Jacobian directly references only $x_t$ and $m_2$, the EKF
correction can still update $m_1$ and $m_3$ through their covariance with the
observed variables.

## EKF correction

After associating observation $z_t^j$ with landmark $j$, calculate

$$
\nu_t=z_t^j-h_j(\bar\mu_t),
$$

$$
S_t=H_t^j\bar\Sigma_t(H_t^j)^T+Q_t,
$$

$$
K_t=\bar\Sigma_t(H_t^j)^TS_t^{-1}.
$$

Here,

* $\nu_t$ is the innovation, the difference between actual and predicted
  observation;
* $S_t$ is the innovation covariance;
* $K_t$ is the Kalman gain.

For range-bearing observations,

$$
\nu_t=
\begin{bmatrix}
r_t-\hat r_t\\
\phi_t-\hat\phi_t
\end{bmatrix}.
$$

Normalize the bearing component of $\nu_t$ to $[-\pi,\pi)$, then update:

$$
\mu_t=\bar\mu_t+K_t\nu_t,
$$

$$
\Sigma_t=(I-K_tH_t^j)\bar\Sigma_t.
$$

Although $H_t^j$ directly references only the robot and one landmark, the
Kalman gain contains the complete covariance. The correction can therefore
change the entire joint state.

### Why one observation updates the whole map

Partition the Kalman gain into robot and landmark rows:

$$
K_t=
\begin{bmatrix}
K_x\\
K_{m_1}\\
\vdots\\
K_{m_N}
\end{bmatrix}.
$$

The mean update becomes

$$
\begin{bmatrix}
\mu_x\\
\mu_{m_1}\\
\vdots\\
\mu_{m_N}
\end{bmatrix}
=
\begin{bmatrix}
\bar\mu_x\\
\bar\mu_{m_1}\\
\vdots\\
\bar\mu_{m_N}
\end{bmatrix}
+
\begin{bmatrix}
K_x\\
K_{m_1}\\
\vdots\\
K_{m_N}
\end{bmatrix}
\nu_t.
$$

An unobserved landmark $i$ can have $K_{m_i}\ne0$ because

$$
K_t=\bar\Sigma_t(H_t^j)^TS_t^{-1}
$$

contains the cross-covariances between landmark $i$, the robot, and observed
landmark $j$. If those cross-covariances were zero, the unobserved landmark
would not be corrected.

This behavior is a defining property of EKF-SLAM:

> A local landmark observation can produce a global map correction because
> the filter maintains correlations among all state variables.

### Innovation covariance

The innovation covariance

$$
S_t=H_t^j\bar\Sigma_t(H_t^j)^T+Q_t
$$

combines two sources of uncertainty:

* uncertainty in the predicted robot and landmark state, projected into
  measurement space by $H_t^j$;
* physical sensor and feature-extraction uncertainty represented by $Q_t$.

For a range-bearing sensor,

$$
S_t\in\mathbb R^{2\times2}.
$$

Therefore, computing $S_t^{-1}$ is inexpensive. The expensive part is
updating the large, dense joint covariance.

## EKF-SLAM algorithm

Input:

$$
\mu_{t-1},\quad
\Sigma_{t-1},\quad
u_t,\quad
z_t.
$$

### Prediction

1. Predict the robot pose using the nonlinear motion model.
2. Copy all static landmark means unchanged.
3. Evaluate the augmented motion Jacobian.
4. Predict the complete joint covariance.

$$
\bar\mu_t=
g^{\text{SLAM}}(u_t,\mu_{t-1})
$$

$$
\bar\Sigma_t=
G_t\Sigma_{t-1}G_t^T+R_t^{\text{aug}}.
$$

### Association and map management

For each detected feature:

1. Compare the observation with predicted observations of compatible
   landmarks.
2. Associate it with an existing landmark if the match is sufficiently
   likely.
3. Otherwise, initialize a new landmark if the observation is suitable.

This stage decides which observation function $h_j$ and Jacobian $H_t^j$ the
correction must use.

### Correction

For an observation associated with landmark $j$:

$$
\hat z_t^j=h_j(\bar\mu_t)
$$

$$
\nu_t=z_t^j-\hat z_t^j
$$

$$
S_t=H_t^j\bar\Sigma_t(H_t^j)^T+Q_t
$$

$$
K_t=\bar\Sigma_t(H_t^j)^TS_t^{-1}
$$

$$
\mu_t=\bar\mu_t+K_t\nu_t
$$

$$
\Sigma_t=(I-K_tH_t^j)\bar\Sigma_t.
$$

When multiple landmarks are observed in one scan, the measurements may be
processed sequentially or stacked into one larger correction. Sequential
updates are simpler to implement. A stacked update preserves the correlations
among measurements when their joint noise model is known.

The complete flow is:

```text
previous robot-map belief
          |
          v
motion prediction using odometry/control
          |
          v
predicted robot-map belief
          |
          v
detect features and perform data association
          |
          +---- new feature ----> initialize landmark
          |
          +---- known feature --> EKF correction
          |
          v
corrected robot-map belief
```

## Landmark initialization

If an observation does not match an existing landmark, it may represent a new
landmark. Given robot pose $(x_t^r,y_t^r,\theta_t)$ and a range-bearing
measurement $(r,\phi)$, initialize its map position as

$$
m_{\text{new}}=
\begin{bmatrix}
x_t^r+r\cos(\theta_t+\phi)\\
y_t^r+r\sin(\theta_t+\phi)
\end{bmatrix}.
$$

The state vector and covariance must then be augmented. The new landmark's
uncertainty is not independent: it must include correlations with the robot
and existing landmarks because its position was calculated using the
uncertain robot pose.

### Inverse observation model

Landmark initialization uses the inverse sensor model

$$
m_{\text{new}}=h^{-1}(x_t,z_t).
$$

For range and bearing,

$$
h^{-1}(x_t,z_t)
=
\begin{bmatrix}
x_t^r+r\cos(\theta_t+\phi)\\
y_t^r+r\sin(\theta_t+\phi)
\end{bmatrix}.
$$

Linearize this inverse model with respect to the robot pose and measurement:

$$
J_x=
\frac{\partial h^{-1}}{\partial x}
=
\begin{bmatrix}
1 & 0 & -r\sin(\theta_t+\phi)\\
0 & 1 & r\cos(\theta_t+\phi)
\end{bmatrix}
$$

and

$$
J_z=
\frac{\partial h^{-1}}{\partial z}
=
\begin{bmatrix}
\cos(\theta_t+\phi) & -r\sin(\theta_t+\phi)\\
\sin(\theta_t+\phi) & r\cos(\theta_t+\phi)
\end{bmatrix}.
$$

The new landmark covariance is approximately

$$
\Sigma_{\text{new,new}}
=
J_x\Sigma_{xx}J_x^T
+
J_zQ_tJ_z^T.
$$

Its cross-covariance with the existing joint state is

$$
\Sigma_{y,\text{new}}
=
\Sigma_{y,x}J_x^T.
$$

Therefore, the augmented covariance has the structure

$$
\Sigma_t^{\text{aug}}
=
\begin{bmatrix}
\Sigma_t & \Sigma_{y,\text{new}}\\
\Sigma_{y,\text{new}}^T & \Sigma_{\text{new,new}}
\end{bmatrix}.
$$

Initializing the new cross-covariance as zero would incorrectly claim that
the new landmark is independent of the uncertain robot pose used to create
it.

## Data association

Data association answers:

> Which existing landmark generated this observation?

This is one of the hardest parts of landmark-based SLAM. A wrong association
can produce a large incorrect correction and cause the EKF to diverge.

A common approach is nearest-neighbor association with Mahalanobis-distance
gating:

$$
d_j^2
=
\nu_j^T
S_j^{-1}
\nu_j,
$$

where

$$
\nu_j=z_t-h_j(\bar\mu_t)
$$

and

$$
S_j=H_j\bar\Sigma_tH_j^T+Q_t.
$$

Associate the observation with the compatible landmark having the smallest
$d_j^2$. If no landmark passes the gate, initialize a new landmark.

Mahalanobis distance differs from ordinary Euclidean distance because it
accounts for uncertainty. A residual of one meter may be plausible for an
uncertain landmark but implausible for a precisely known landmark.

For a two-dimensional range-bearing observation, a gate can be selected from
a chi-squared distribution with two degrees of freedom:

$$
d_j^2 < \chi^2_{2,\alpha}.
$$

The confidence level $\alpha$ controls how permissive the gate is. A loose
gate accepts more true matches but also increases the risk of false
associations. A strict gate rejects more false matches but may create duplicate
landmarks.

### Known and unknown correspondence

If landmarks have unique identifiers, such as coded visual markers or
reflectors with distinguishable signatures, correspondence may be known:

$$
z_t^i \longleftrightarrow m_j.
$$

With natural geometric features, correspondence is usually unknown and must
be inferred. Repeated structures such as similar corners, pillars, or shelves
make this difficult.

The EKF assumes the selected association is correct. It does not represent
multiple competing associations. This is dangerous because one confident
incorrect match can pull the robot and the entire correlated map toward a
wrong configuration.

More advanced association methods include:

* joint compatibility branch and bound;
* multiple-hypothesis tracking;
* probabilistic data association;
* rejecting ambiguous observations instead of forcing a match.

In practice, robust association is often more important than making the
motion model slightly more accurate.

## Getting landmarks from a production 2D LiDAR

A 2D LiDAR normally outputs a scan containing range measurements at known
angles:

$$
z=\{(r_i,\alpha_i)\}_{i=1}^{M}.
$$

It does not directly output the abstract point landmarks assumed by the
EKF-SLAM model. Each valid return can first be converted into a point in the
LiDAR frame:

$$
p_i^L=
\begin{bmatrix}
r_i\cos\alpha_i\\
r_i\sin\alpha_i
\end{bmatrix}.
$$

A feature-extraction front end must then turn scan points into repeatable
landmark observations. Examples include:

* detecting high-intensity reflective markers;
* clustering returns from poles and fitting their centers;
* fitting wall lines and using stable corners or line endpoints;
* detecting other distinctive geometric features.

The feature detector must provide both a landmark measurement and a reasonable
measurement-noise covariance $Q_t$. Features must remain detectable from
different robot poses for data association to work reliably.

In controlled industrial environments, installed reflective markers can make
landmark-based localization highly reliable. In ordinary environments,
natural point landmarks extracted from 2D scans may be unstable because of
occlusion, viewpoint changes, moving objects, and repeated geometry.

### Example: extracting a pole landmark

Suppose a scan contains a compact cluster of returns from a cylindrical pole.
A possible front-end pipeline is:

```text
raw ranges
   |
   v
remove invalid and out-of-range returns
   |
   v
cluster neighboring scan points
   |
   v
reject clusters with unsuitable size or shape
   |
   v
fit a circle or estimate the cluster center
   |
   v
produce range-bearing landmark observation and covariance
```

If the estimated pole center in the LiDAR frame is

$$
p^L=
\begin{bmatrix}
p_x^L\\p_y^L
\end{bmatrix},
$$

the corresponding range-bearing observation is

$$
z=
\begin{bmatrix}
\sqrt{(p_x^L)^2+(p_y^L)^2}\\
\operatorname{atan2}(p_y^L,p_x^L)
\end{bmatrix}.
$$

This observation can enter the EKF-SLAM range-bearing model. Its covariance
should reflect LiDAR range noise, angular resolution, cluster geometry, and
feature-fitting error.

### Sensor frame and robot frame

The LiDAR is rarely mounted exactly at the robot pose origin. If its calibrated
extrinsic transform is $T_R^L$, a point measured in the LiDAR frame must be
transformed into the robot frame:

$$
p^R=T_R^L p^L.
$$

Ignoring this transform creates a systematic observation error. The EKF may
incorrectly interpret that calibration error as robot-pose or landmark error.

### Dynamic and temporary objects

Returns from people, vehicles, doors, pallets, and other movable objects
should generally not become permanent landmarks. Production front ends use
geometric checks, temporal consistency, semantic filtering, or repeated
observations to avoid inserting transient objects into the map.

## ICP and EKF-SLAM are different components

Iterative Closest Point (ICP) aligns two point clouds and estimates the rigid
transformation between them:

$$
T^*
=
\arg\min_T
\sum_i \|q_i-Tp_i\|^2.
$$

For consecutive 2D LiDAR scans, ICP can estimate relative robot motion:

$$
\Delta x,\quad\Delta y,\quad\Delta\theta.
$$

ICP is a scan-matching method. It does not inherently maintain the landmark
state and joint covariance used by landmark-based EKF-SLAM.

ICP repeatedly performs two main operations:

1. Match points in one scan with points or surfaces in another scan or map.
2. Solve for the rigid transformation that reduces alignment error.

The estimated transformation can be used as:

* relative motion between consecutive scans;
* a pose observation obtained by matching against a fixed map;
* a constraint between poses in a graph-based SLAM system.

The role of ICP depends on the surrounding estimator. Calling a system
"EKF-ICP" does not by itself imply that landmarks are included in the EKF
state.

There are several possible system designs:

| Design | EKF state | Map handling |
| --- | --- | --- |
| Landmark EKF-SLAM | Robot pose and explicit landmarks | Landmarks are inside the EKF state |
| EKF with ICP measurement | Robot pose, velocity, biases, etc. | ICP supplies relative-pose corrections; map may be separate |
| Scan-matching localization | Robot pose only | Current scan is matched against a fixed map |
| Graph-based LiDAR SLAM | Pose nodes, constraints, and possibly landmarks | Scan matching creates constraints; graph optimization builds a consistent map |

If ICP is used only to produce a relative-pose measurement for an EKF, the EKF
usually does not predict explicit landmark positions. Instead, it predicts the
robot state using a motion model and corrects that state using the ICP result.
The point-cloud or occupancy-grid map is maintained separately.

However, ICP alone is not a complete global SLAM solution. It is a local
alignment method and can accumulate drift, converge to the wrong alignment,
or fail in environments with insufficient geometry. A complete SLAM system
typically also needs map management, loop-closure detection, and global
optimization or another mechanism for maintaining consistency.

For example, scan-to-scan ICP estimates a chain of relative motions:

$$
T_{0,1},\quad T_{1,2},\quad \ldots,\quad T_{t-1,t}.
$$

Composing these transformations estimates the trajectory:

$$
T_{0,t}
=
T_{0,1}T_{1,2}\cdots T_{t-1,t}.
$$

Small errors in every relative transformation accumulate over time. This is
drift. Recognizing a previously visited place and adding a loop-closure
constraint allows a global estimator to reduce that accumulated error.

## Mapping versus localization in production

SLAM means that the system estimates both the robot trajectory and the map:

$$
p(x_{1:t},m\mid z_{1:t},u_{1:t}).
$$

When a robot operates using an already known and fixed map, it estimates only
its pose:

$$
p(x_t\mid z_{1:t},u_{1:t},m).
$$

That process is **localization**, not SLAM.

Many production deployments deliberately separate these activities:

```text
Dedicated mapping phase:
sensor data -> SLAM or surveying -> map cleanup and validation -> fixed map

Normal operation:
fixed map + live sensor data + odometry -> localization
```

The offline map may be created using landmark EKF-SLAM, LiDAR scan matching,
graph-based SLAM, surveying, or a combination of techniques. During normal
operation, keeping the map fixed generally makes behavior easier to validate
and prevents transient objects or localization errors from corrupting it.

During localization against a fixed map, a typical estimator may maintain
only a robot state such as

$$
\mu_t=
\begin{bmatrix}
x_t^r&y_t^r&\theta_t&v_t&\omega_t&\cdots
\end{bmatrix}^T.
$$

The map remains an external input rather than a random variable inside the
filter. LiDAR scan matching, landmark matching, Monte Carlo localization, or
another localization method compares live measurements against that map.

This distinction can be tested with one question:

> Does the estimator update the map uncertainty or map state during normal
> operation?

If the map is fixed and only the robot pose is updated, the process is
localization. If both pose and map are estimated or updated, it is a form of
SLAM or online mapping.

This separation is common, but not universal. A robot may still need online
SLAM or map updates when:

* no prior map exists;
* the environment changes significantly;
* the robot explores new areas;
* long-term map maintenance is required.

Therefore, a precise production-oriented statement is:

> In SLAM, the robot simultaneously estimates its pose and builds or updates a
> map. In many production systems, a map is created during a dedicated mapping
> phase. During normal operation, the map is fixed and the robot performs
> localization only.

## Practical conclusions

* In 2D landmark EKF-SLAM, the joint state has $2N+3$ dimensions because each
  of the $N$ landmarks contains an $x$ and $y$ coordinate.
* Prediction moves the robot state; static landmark means remain unchanged.
* Observing one landmark can update the complete state because all estimates
  are correlated through the joint covariance.
* A raw 2D LiDAR scan requires feature extraction before it can be used by
  landmark EKF-SLAM.
* ICP aligns scans or point clouds. It can provide pose corrections without
  placing explicit landmarks inside the EKF state.
* Production systems often use SLAM during a dedicated mapping phase and
  localization against a fixed map during normal operation.

## Properties and limitations

EKF-SLAM represents the complete robot-map belief using one Gaussian:

$$
p(x_t,m\mid z_{1:t},u_{1:t})
\approx
\mathcal N(\mu_t,\Sigma_t).
$$

This representation is useful because it explicitly maintains uncertainty and
correlation. However, it also imposes important limitations.

EKF-SLAM works best when:

* the map contains a manageable number of stable landmarks;
* motion and observation uncertainty are not too large;
* the current estimate is close enough for local linearization to be valid;
* data association is reliable;
* noise covariances and sensor calibration are realistic.

It can fail or become inconsistent when:

* linearization errors accumulate;
* the initial robot pose or landmark positions are poor;
* observations are incorrectly associated;
* dynamic objects are treated as static landmarks;
* uncertainty is underestimated;
* the dense covariance becomes too expensive for the map size.

The state dimension grows linearly with the number of point landmarks:

$$
\dim(\mu_t)=2N+3.
$$

The dense covariance contains approximately the square of that number of
entries:

$$
\operatorname{entries}(\Sigma_t)=(2N+3)^2.
$$

Consequently, classic EKF-SLAM becomes expensive for large environments. This
motivates sparse information filters, submapping, particle-filter methods,
and modern graph-based SLAM approaches.

In short:

$$
\boxed{
\text{EKF-SLAM}
=
\text{EKF}
+
\text{joint robot-map state}
+
\text{landmark initialization}
+
\text{data association}
}
$$
