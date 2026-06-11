# Extended Kalman Filter

[Kalman Filter and Extended Kalman Filter (EKF)](http://ais.informatik.uni-freiburg.de/teaching/ws22/mapping/)

## Recall: limitation of the Kalman filter

The Kalman filter assumes that the motion and observation models are linear:

$$
x_t=A_t x_{t-1}+B_tu_t+\epsilon_t
$$

$$
z_t=C_tx_t+\delta_t
$$

with Gaussian noise

$$
\epsilon_t\sim\mathcal N(0,R_t),
\qquad
\delta_t\sim\mathcal N(0,Q_t).
$$

However, most robotics models are nonlinear. For example, a robot moving with
heading $\theta$ contains terms such as $\cos\theta$ and $\sin\theta$, and a
range-bearing sensor contains $\sqrt{\cdot}$ and $\operatorname{atan2}$.

The **extended Kalman filter (EKF)** handles these models by replacing each
nonlinear function with a local linear approximation. It then applies the
Kalman-filter equations to that approximation.

The main idea is:

$$
\text{nonlinear function}
\xrightarrow{\text{first-order Taylor expansion}}
\text{locally linear function}.
$$

> [!NOTE]
> 
> The EKF does not make a nonlinear system linear everywhere. It only
> approximates the system near the current estimate.

## Nonlinear model

Replace the Kalman filter's linear motion model with a nonlinear function
$g$:

$$
x_t=g(u_t,x_{t-1})+\epsilon_t,
\qquad
\epsilon_t\sim\mathcal N(0,R_t).
$$

Therefore,

$$
p(x_t\mid u_t,x_{t-1})
=
\mathcal N(g(u_t,x_{t-1}),R_t).
$$

Replace the linear observation model with a nonlinear function $h$:

$$
z_t=h(x_t)+\delta_t,
\qquad
\delta_t\sim\mathcal N(0,Q_t).
$$

Therefore, observation likelihood (sensor model), conditional probability density function (PDF).

> Given that the system is in state $x_t$, how likely is the sensor to produce measurement $z_t$?

$$
p(z_t\mid x_t)
=
\mathcal N(h(x_t),Q_t).
$$

| Model | Kalman filter | Extended Kalman filter |
| --- | --- | --- |
| Motion | $A_tx_{t-1}+B_tu_t$ | $g(u_t,x_{t-1})$ |
| Observation | $C_tx_t$ | $h(x_t)$ |
| Linearization matrix | $A_t,\ C_t$ | Jacobians $G_t,\ H_t$ |

## First-order Taylor expansion

For a scalar function $f(x)$, the first-order Taylor expansion around a point
$\mu$ is

$$
f(x)\approx f(\mu)+f'(\mu)(x-\mu).
$$

This approximation is a line tangent to $f$ at $\mu$. It is accurate near
$\mu$, but its error generally increases farther away from $\mu$.

For a vector-valued function, the derivative is replaced by a **Jacobian
matrix**:

$$
f(x)\approx f(\mu)+J_f(\mu)(x-\mu).
$$

This is the local linearization used by the EKF.

## Jacobian matrix

Suppose

$$
f(x)=
\begin{bmatrix}
f_1(x_1,\ldots,x_n)\\
\vdots\\
f_m(x_1,\ldots,x_n)
\end{bmatrix}.
$$

Its Jacobian is the $m\times n$ matrix

$$
J_f(x)
=
\frac{\partial f}{\partial x}
=
\begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n}\\
\vdots & \ddots & \vdots\\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix}.
$$

Each entry describes how one output component changes with respect to one
input component.

For the EKF, define the motion-model Jacobian

$$
G_t
=
\left.
\frac{\partial g(u_t,x)}{\partial x}
\right|_{x=\mu_{t-1}}
$$

and the observation-model Jacobian

$$
H_t
=
\left.
\frac{\partial h(x)}{\partial x}
\right|_{x=\bar\mu_t}.
$$

The two Jacobians are evaluated at different points:

* $G_t$ is evaluated at the previous corrected mean $\mu_{t-1}$.
* $H_t$ is evaluated at the predicted mean $\bar\mu_t$.

## Linearizing the motion model

The nonlinear motion function is

$$
x_t=g(u_t,x_{t-1})+\epsilon_t.
$$

Linearize $g$ around the previous mean $\mu_{t-1}$:

$$
g(u_t,x_{t-1})
\approx
g(u_t,\mu_{t-1})
+
G_t(x_{t-1}-\mu_{t-1}).
$$

Substitute this approximation into the nonlinear motion model:

$$
x_t
\approx
g(u_t,\mu_{t-1})
+
G_t(x_{t-1}-\mu_{t-1})
+
\epsilon_t.
$$

Because

$$
\epsilon_t\sim\mathcal N(0,R_t),
$$

the linearized conditional motion model is approximately Gaussian:

$$
p(x_t\mid u_t,x_{t-1})
\approx
\mathcal N\left(
x_t;\,
g(u_t,\mu_{t-1})+G_t(x_{t-1}-\mu_{t-1}),\,
R_t
\right).
$$

Equivalently, its probability density function is

$$
\begin{aligned}
p(x_t\mid u_t,x_{t-1})
&\approx
\det(2\pi R_t)^{-\frac{1}{2}}\\
&\quad
\exp\left[
-\frac{1}{2}
\left(
x_t-g(u_t,\mu_{t-1})-G_t(x_{t-1}-\mu_{t-1})
\right)^T
R_t^{-1}
\left(
x_t-g(u_t,\mu_{t-1})-G_t(x_{t-1}-\mu_{t-1})
\right)
\right].
\end{aligned}
$$

The residual

$$
x_t-g(u_t,\mu_{t-1})-G_t(x_{t-1}-\mu_{t-1})
$$

is the motion error under the linearized model. The approximation symbol is
needed because the original function $g$ is nonlinear. The covariance $R_t$
still describes the motion noise.

Taking the expected value gives the predicted mean:

$$
\bar\mu_t=g(u_t,\mu_{t-1}).
$$

The constant term does not contribute to covariance. The locally linear term
transforms the old covariance, and the motion noise adds uncertainty:

$$
\bar\Sigma_t
=
G_t\Sigma_{t-1}G_t^T+R_t.
$$

This has the same structure as the Kalman-filter prediction:

$$
\bar\Sigma_t=A_t\Sigma_{t-1}A_t^T+R_t,
$$

but the fixed linear matrix $A_t$ is replaced by the local Jacobian $G_t$.

## Linearizing the observation model

The nonlinear observation function is

$$
z_t=h(x_t)+\delta_t.
$$

Linearize $h$ around the predicted mean $\bar\mu_t$:

$$
h(x_t)
\approx
h(\bar\mu_t)+H_t(x_t-\bar\mu_t).
$$

Substitute this approximation into the nonlinear observation model:

$$
z_t
\approx
h(\bar\mu_t)
+
H_t(x_t-\bar\mu_t)
+
\delta_t.
$$

Because

$$
\delta_t\sim\mathcal N(0,Q_t),
$$

the linearized observation likelihood is approximately Gaussian:

$$
p(z_t\mid x_t)
\approx
\mathcal N\left(
z_t;\,
h(\bar\mu_t)+H_t(x_t-\bar\mu_t),\,
Q_t
\right).
$$

Equivalently, its probability density function is

$$
\begin{aligned}
p(z_t\mid x_t)
&\approx
\det(2\pi Q_t)^{-\frac{1}{2}}\\
&\quad
\exp\left[
-\frac{1}{2}
\left(
z_t-h(\bar\mu_t)-H_t(x_t-\bar\mu_t)
\right)^T
Q_t^{-1}
\left(
z_t-h(\bar\mu_t)-H_t(x_t-\bar\mu_t)
\right)
\right].
\end{aligned}
$$

The residual

$$
z_t-h(\bar\mu_t)-H_t(x_t-\bar\mu_t)
$$

is the measurement error under the linearized model. The covariance $Q_t$
still describes the observation noise.

The predicted observation is

$$
\hat z_t=h(\bar\mu_t).
$$

At the linearization point $x_t=\bar\mu_t$, the deviation term is zero:

$$
H_t(x_t-\bar\mu_t)=0.
$$

Therefore, the predicted measurement at the current estimate is
$h(\bar\mu_t)$.

The innovation, or measurement residual, is

$$
y_t=z_t-\hat z_t=z_t-h(\bar\mu_t).
$$

The innovation covariance is

$$
S_t=H_t\bar\Sigma_tH_t^T+Q_t.
$$

It combines the predicted state uncertainty, projected into measurement space,
with the sensor noise.

The Kalman gain is

$$
K_t=\bar\Sigma_tH_t^TS_t^{-1}
=
\bar\Sigma_tH_t^T
(H_t\bar\Sigma_tH_t^T+Q_t)^{-1}.
$$

Use the innovation to correct the predicted mean:

$$
\mu_t
=
\bar\mu_t+K_t\left(z_t-h(\bar\mu_t)\right).
$$

Then update the covariance:

$$
\Sigma_t=(I-K_tH_t)\bar\Sigma_t.
$$

Again, these equations have the same structure as the Kalman filter, but
$C_t\bar\mu_t$ is replaced by $h(\bar\mu_t)$ and $C_t$ is replaced by $H_t$.

## Extended Kalman filter algorithm

Input:

$$
\mu_{t-1},\ \Sigma_{t-1},\ u_t,\ z_t.
$$

### Prediction

Evaluate the motion Jacobian:

$$
G_t
=
\left.
\frac{\partial g(u_t,x)}{\partial x}
\right|_{x=\mu_{t-1}}.
$$

Predict the state mean and covariance:

$$
\bar\mu_t=g(u_t,\mu_{t-1})
$$

$$
\bar\Sigma_t=G_t\Sigma_{t-1}G_t^T+R_t.
$$

### Correction

Evaluate the observation Jacobian:

$$
H_t
=
\left.
\frac{\partial h(x)}{\partial x}
\right|_{x=\bar\mu_t}.
$$

Compute the innovation, its covariance, and the Kalman gain:

$$
y_t=z_t-h(\bar\mu_t)
$$

$$
S_t=H_t\bar\Sigma_tH_t^T+Q_t
$$

$$
K_t=\bar\Sigma_tH_t^TS_t^{-1}.
$$

Correct the state mean and covariance:

$$
\mu_t=\bar\mu_t+K_ty_t
$$

$$
\Sigma_t=(I-K_tH_t)\bar\Sigma_t.
$$

Output:

$$
\mu_t,\ \Sigma_t.
$$

The complete algorithm can be summarized as

$$
\boxed{
\begin{aligned}
\bar\mu_t &= g(u_t,\mu_{t-1})\\
\bar\Sigma_t &= G_t\Sigma_{t-1}G_t^T+R_t\\
K_t &= \bar\Sigma_tH_t^T(H_t\bar\Sigma_tH_t^T+Q_t)^{-1}\\
\mu_t &= \bar\mu_t+K_t(z_t-h(\bar\mu_t))\\
\Sigma_t &= (I-K_tH_t)\bar\Sigma_t.
\end{aligned}}
$$

## Example: range-bearing observation

Consider a robot pose

$$
x=
\begin{bmatrix}
x_r\\y_r\\\theta
\end{bmatrix}
$$

and a known landmark at

$$
m=
\begin{bmatrix}
m_x\\m_y
\end{bmatrix}.
$$

Define

$$
\Delta x=m_x-x_r,
\qquad
\Delta y=m_y-y_r,
\qquad
q=\Delta x^2+\Delta y^2.
$$

The nonlinear sensor predicts the range and bearing:

$$
h(x)
=
\begin{bmatrix}
\sqrt q\\
\operatorname{atan2}(\Delta y,\Delta x)-\theta
\end{bmatrix}.
$$

Its Jacobian with respect to the robot state is

$$
H
=
\frac{\partial h}{\partial x}
=
\begin{bmatrix}
-\frac{\Delta x}{\sqrt q} &
-\frac{\Delta y}{\sqrt q} &
0\\
\frac{\Delta y}{q} &
-\frac{\Delta x}{q} &
-1
\end{bmatrix}.
$$

The EKF uses $h(\bar\mu_t)$ to predict what the sensor should observe and uses
$H$ to approximate how state uncertainty affects that observation.

Because bearing is an angle, its innovation must be normalized, usually to
$[-\pi,\pi)$, before applying the correction:

$$
y_{\text{bearing}}
\leftarrow
\operatorname{normalizeAngle}(y_{\text{bearing}}).
$$

Without angle normalization, two nearly identical directions such as
$-\pi+\varepsilon$ and $\pi-\varepsilon$ appear to have a residual close to
$2\pi$.

## Relation to the Kalman filter

| Step | Kalman filter | Extended Kalman filter |
| --- | --- | --- |
| Predict mean | $A_t\mu_{t-1}+B_tu_t$ | $g(u_t,\mu_{t-1})$ |
| Predict covariance | $A_t\Sigma_{t-1}A_t^T+R_t$ | $G_t\Sigma_{t-1}G_t^T+R_t$ |
| Predicted observation | $C_t\bar\mu_t$ | $h(\bar\mu_t)$ |
| Observation matrix | $C_t$ | $H_t$ |
| Innovation | $z_t-C_t\bar\mu_t$ | $z_t-h(\bar\mu_t)$ |

If $g$ and $h$ are linear, their Jacobians are the original linear matrices:

$$
G_t=A_t,
\qquad
H_t=C_t.
$$

In that case, the EKF equations reduce to the ordinary Kalman-filter
equations.

## Properties and limitations

The EKF keeps only a Gaussian approximation of the belief:

$$
bel(x_t)\approx\mathcal N(\mu_t,\Sigma_t).
$$

A nonlinear transformation of a Gaussian is generally **not Gaussian**.
Therefore, unlike the linear Kalman filter, the EKF is not an exact Bayes
filter and is not guaranteed to be optimal.

The approximation works well when:

* the nonlinear functions are smooth;
* uncertainty is small enough that the functions are nearly linear over the
  likely state region;
* the current estimate is close to the true state;
* the Jacobians and noise covariances are modeled correctly.

The approximation can fail when:

* the model is strongly nonlinear;
* uncertainty is large;
* the estimate is far from the true state;
* an incorrect measurement or data association causes a bad correction.

Because every new linearization is centered on the current estimate, a poor
estimate can produce a poor Jacobian, which can cause the filter to become
inconsistent or diverge.

In short:

$$
\text{EKF}
=
\text{Kalman filter}
+
\text{local first-order linearization}.
$$
