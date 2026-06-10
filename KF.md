# Kalman Filter

## recall

SLAM is state estimation problem.

And to estimate the map and robot's pose,

We adopt a tool, Bayes filter, for state estimation.



* prediction step:

  * $$
    \overline{bel}(x_t)=
    \int p(x_t|x_{t-1},u_{t}) bel(x_{t-1}) dx_{t-1}
    $$

  * $p(x_t|x_{t-1},u_{t})$ is defined as **motion model**

* correction step:

  * $$
    bel(x_t)=\eta p(z_t|x_t)\overline{bel}(x_t)
    $$

  * $p(z_t|x_t)$ is defined as sensor or observation model

## Feature of Kalman Filter

* Kalman filter is one of Bayes filter.
* Estimator based on linear Gaussian assumption
* Therefore is optimal solution for linear models and Normal(Gaussian) distributions.

## Gaussian distribution and its property

[Multivariate normal distribution](https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Non-degenerate_case)

![](https://wikimedia.org/api/rest_v1/media/math/render/svg/418f7f0a42778c344e9bff0876adb348655cd367)



### property

$\Sigma$​ (Sigma) mean **covariance matrix** of Gaussian distribution.

And covariance matrix generalizes **variance** concept to multiple dimensions.
$$
p(x)=p(x_a,x_b)=\mathcal{N}(\mu,\Sigma)
$$
where $\mu=\begin{pmatrix} \mu_a \\ \mu_b \end{pmatrix}$ and $\Sigma=\begin{pmatrix} \Sigma_{aa} & \Sigma_{ab} \\ \Sigma_{ba} & \Sigma_{bb} \end{pmatrix}$

#### marginalization



#### conditioning



## Linear Model

recall state space (modern control)
$$
x_t=A_t x_{t-1}+B_t u_t +\epsilon_t \\
z_t = C_t x_t + \delta_t
$$

| Control lecture                       | SLAM / Kalman filter slide | Meaning                                     |
| ------------------------------------- | -------------------------- | ------------------------------------------- |
| $x_t$                                 | $x_t$                      | hidden system state                         |
| $u_t$                                 | $u_t$                      | control input, such as robot motion command |
| $y_t$                                 | $z_t$                      | measurement / observation                   |
| $A$                                   | $A_t$                      | state transition matrix                     |
| $B$                                   | $B_t$                      | control-input matrix                        |
| $C$                                   | $C_t$                      | observation matrix                          |
| $D$                                   | omitted                    | direct feedthrough from input to output     |
| usually omitted in basic control form | $\epsilon_t,\delta_t$      | process noise and measurement noise         |

The main reason you see no $D u_t$ term is that in many SLAM/Kalman-filter models, the measurement is assumed to depend on the **current state**, not directly on the control input:
$$
z_t=C_tx_t+\delta_t
$$


For example, a robot’s laser scanner or camera measurement depends on the robot pose and map, not directly on the wheel command $u_t$. The control command first changes the robot state through
$$
x_t=A_tx_{t-1}+B_tu_t+\epsilon_t
$$


and then the sensor observes that state.

So the effect of $u_t$ on $z_t$ is usually **indirect**:
$$
u_t \rightarrow x_t \rightarrow z_t
$$

## motion model

$$
p(x_t|u_t,x_{t-1})=?
$$

---

Start from the linear state transition model:
$$
x_t = A_t x_{t-1} + B_t u_t + \epsilon_t
$$
where
$$
\epsilon_t \sim \mathcal N(0, R_t)
$$


That means the motion noise is a zero-mean multivariate Gaussian with covariance matrix $R_t$.

------

### 1. Interpret the motion equation

The deterministic part of the motion is:
$$
A_t x_{t-1} + B_t u_t
$$
So without noise, the next state would be exactly:

$$
x_t = A_t x_{t-1} + B_t u_t
$$

But in reality, the robot’s motion is noisy. Wheels slip, motors are imperfect, the floor is uneven, etc. So we add noise:

$$
x_t = A_t x_{t-1} + B_t u_t + \epsilon_t
$$

Rearrange this equation to isolate the noise:

$$
\epsilon_t = x_t - A_t x_{t-1} - B_t u_t
$$

This is the key step.

The probability of ending at state $x_t$, given the previous state $x_{t-1}$ and control $u_t$, is the same as the probability that the noise took exactly the value needed to make that happen.

So:

$$
p(x_t \mid u_t, x_{t-1})

p(\epsilon_t = x_t - A_t x_{t-1} - B_t u_t)
$$

------

### 2. Use the multivariate Gaussian density

For an (n)-dimensional Gaussian random variable

$$
\epsilon \sim \mathcal N(\mu, \Sigma)
$$

its probability density is:

$$
p(\epsilon)

\frac{1}{\sqrt{(2\pi)^n \det(\Sigma)}}
\exp
\left[
-\frac{1}{2}
(\epsilon-\mu)^T
\Sigma^{-1}
(\epsilon-\mu)
\right]
$$

Here, the slide assumes:

$$
\epsilon_t \sim \mathcal N(0, R_t)
$$

So:

$$
\mu = 0
$$

and

$$
\Sigma = R_t
$$

Therefore:

$$
p(\epsilon_t)
=
\frac{1}{\sqrt{(2\pi)^n \det(R_t)}}
\exp
\left[
-\frac{1}{2}
\epsilon_t^T R_t^{-1} \epsilon_t
\right]
$$

This normalization term can also be written as:

$$
\det(2\pi R_t)^{-\frac{1}{2}}
$$

because:

$$
\det(2\pi R_t)
=
(2\pi)^n \det(R_t)
$$

So:
$$
\frac{1}{\sqrt{(2\pi)^n \det(R_t)}}
=
\det(2\pi R_t)^{-\frac{1}{2}}
$$
------

### 3. Substitute the required noise value

From the motion equation:
$$
\epsilon_t = x_t - A_t x_{t-1} - B_t u_t
$$
Substitute this into the Gaussian density:
$$
p(x_t \mid u_t, x_{t-1})
=
\det(2\pi R_t)^{-\frac{1}{2}}
\exp
\left[
-\frac{1}{2}
(x_t - A_t x_{t-1} - B_tu_t)^T
R_t^{-1}
(x_t - A_t x_{t-1} - B_tu_t)
\right]
$$
That is exactly the formula on the slide.

------

### 4. Meaning of the formula

The conditional distribution is:
$$
x_t \mid u_t, x_{t-1}
\sim
\mathcal N(A_t x_{t-1} + B_t u_t, R_t)
$$
So the **mean** of the next state is:
$$
\mathbb E[x_t \mid u_t, x_{t-1}]
=
A_t x_{t-1} + B_t u_t
$$
and the **covariance** is:
$$
\mathrm{Cov}(x_t \mid u_t, x_{t-1})
=
R_t
$$
So $R_t$ controls the spread of possible next states.

Small $R_t$: the robot motion is reliable, so $x_t$ is tightly concentrated near the predicted state.

Large $R_t$: the robot motion is uncertain, so $x_t$ can be far from the predicted state.

------

### 5. Intuition with a simple 1D case

Suppose the state is one-dimensional:
$$
x_t = a x_{t-1} + b u_t + \epsilon_t
$$

with
$$
\epsilon_t \sim \mathcal N(0, \sigma^2)
$$

Then:
$$
x_t \mid u_t, x_{t-1}
\sim
\mathcal N(a x_{t-1} + b u_t, \sigma^2)
$$

The density is:
$$
p(x_t \mid u_t, x_{t-1})
=
\frac{1}{\sqrt{2\pi\sigma^2}}
\exp
\left[
-\frac{1}{2}
\frac{(x_t - ax_{t-1} - bu_t)^2}{\sigma^2}
\right]
$$

This is the scalar version of the slide’s matrix formula.

The term

$$
x_t - A_t x_{t-1} - B_tu_t
$$

is the **prediction error** or **motion residual**.

The exponent says: states $x_t$ that are close to the predicted motion have high probability; states far away have low probability.

So the whole formula is just:
$$
\text{motion probability}
=
\text{Gaussian density of the motion error}
$$

That is the core idea.

## observation model

$$
p(z_t|x_t)=?
$$

---

The observation model is derived almost exactly the same way as the motion model.

The slide starts from the **linear observation equation**:

$$
z_t = C_t x_t + \delta_t
$$

where

$$
\delta_t \sim \mathcal N(0, Q_t)
$$

Here:

$$
x_t
$$

is the hidden state, and

$$
z_t
$$

is the sensor measurement.

For example, in SLAM:

$$
x_t = \text{robot pose, map state, or both}
$$

$$
z_t = \text{laser scan, landmark measurement, camera feature, etc.}
$$

---

### 1. Start from the linear observation model

The noiseless observation would be:

$$
z_t = C_t x_t
$$

That means if the sensor were perfect, the measurement would be exactly the state transformed by $C_t$.

But real sensors are noisy, so we write:

$$
z_t = C_t x_t + \delta_t
$$

where (\delta_t) is measurement noise.

Rearrange to isolate the noise:

$$
\delta_t = z_t - C_t x_t
$$

This quantity is called the **measurement residual** or **innovation**.

It means:

$$
\text{measurement error}
=

\text{actual measurement}
-
\text{predicted measurement}
$$

---

### 2. Assume the measurement noise is Gaussian

The slide assumes:

$$
\delta_t \sim \mathcal N(0, Q_t)
$$

So the noise has mean zero and covariance $Q_t$.

The multivariate Gaussian density is:

$$
p(\delta)
=

\det(2\pi Q_t)^{-\frac{1}{2}}
\exp
\left[
-\frac{1}{2}
\delta^T Q_t^{-1}\delta
\right]
$$

because the mean is zero.

Now substitute:

$$
\delta_t = z_t - C_t x_t
$$

Then:

$$
p(z_t \mid x_t)
=

\det(2\pi Q_t)^{-\frac{1}{2}}
\exp
\left[
-\frac{1}{2}
(z_t-C_tx_t)^TQ_t^{-1}(z_t-C_tx_t)
\right]
$$

That is the formula on the slide.

---

### 3. Why is it $p(z_t \mid x_t)$?

The observation model asks:

> Given that the robot is in state $x_t$, how likely is it that the sensor measures $z_t$?

So the conditional probability is:

$$
p(z_t \mid x_t)
$$

The state $x_t$ is treated as known for this probability calculation.

The randomness comes from the sensor noise $\delta_t$.

Since:

$$
z_t = C_t x_t + \delta_t
$$

and

$$
\delta_t \sim \mathcal N(0,Q_t)
$$

we get:

$$
z_t \mid x_t \sim \mathcal N(C_t x_t, Q_t)
$$

So the mean of the measurement is:

$$
\mathbb E[z_t \mid x_t] = C_t x_t
$$

and the covariance is:

$$
\mathrm{Cov}(z_t \mid x_t) = Q_t
$$

---



### 4. Scalar example

Suppose the measurement is one-dimensional:

$$
z_t = c x_t + \delta_t
$$

with

$$
\delta_t \sim \mathcal N(0,\sigma^2)
$$

Then:

$$
z_t \mid x_t \sim \mathcal N(cx_t,\sigma^2)
$$

The density is:

$$
p(z_t \mid x_t)
=

\frac{1}{\sqrt{2\pi\sigma^2}}
\exp
\left[
-\frac{1}{2}
\frac{(z_t-cx_t)^2}{\sigma^2}
\right]
$$

This is the 1D version of the slide’s matrix equation.

The term

$$
z_t - C_tx_t
$$

measures how surprising the sensor reading is.

If $z_t$ is close to $C_tx_t$, then:

$$
z_t-C_tx_t \approx 0
$$

so the probability is high.

If $z_t$ is far from $C_tx_t$, then the exponent becomes very negative, so the probability is low.

---

### 5. Intuition in SLAM

Imagine the robot believes it is at state (x_t). From that state, it predicts what the sensor should see:

$$
\hat z_t = C_t x_t
$$

Then the real sensor returns:

$$
z_t
$$

The difference is:

$$
z_t-\hat z_t
=

z_t-C_tx_t
$$

The Kalman filter asks:

> Is this measurement consistent with the current state estimate?

The probability is high when the residual is small relative to $Q_t$.

The probability is low when the residual is large relative to $Q_t$.

So $Q_t$ tells the filter how much to trust the measurement.

Small $Q_t$: sensor is reliable, so even small residuals matter a lot.

Large $Q_t$: sensor is noisy, so larger residuals are tolerated.

---

So the observation model formula is simply:

$$
p(z_t \mid x_t)
=

\text{Gaussian density of the measurement error } z_t-C_tx_t
$$

with measurement noise covariance $Q_t$.

## Compare models

The motion model was:

$$
x_t = A_t x_{t-1} + B_tu_t + \epsilon_t
$$

So:

$$
x_t \mid x_{t-1}, u_t
\sim
\mathcal N(A_tx_{t-1}+B_tu_t, R_t)
$$

The observation model is:

$$
z_t = C_t x_t + \delta_t
$$

So:

$$
z_t \mid x_t
\sim
\mathcal N(C_tx_t, Q_t)
$$

They have the same structure:

| Model             |                Mean | Covariance |
| ----------------- | ------------------: | ---------: |
| Motion model      | $A_tx_{t-1}+B_tu_t$ |      $R_t$ |
| Observation model |            $C_tx_t$ |      $Q_t$ |

So the motion model says:

> Given previous state and control, the next state is Gaussian.

The observation model says:

> Given the current state, the sensor measurement is Gaussian.

## Algorithm

we know that
$$
\overline{bel}(x_t)= \int p(x_t|x_{t-1},u_{t}) bel(x_{t-1}) dx_{t-1} \\

bel(x_t)=\eta p(z_t|x_t)\overline{bel}(x_t)
$$

where 
$$
p(x_t \mid u_t, x_{t-1}) = \det(2\pi R_t)^{-\frac{1}{2}} \exp \left[ -\frac{1}{2} (x_t - A_t x_{t-1} - B_tu_t)^T R_t^{-1} (x_t - A_t x_{t-1} - B_tu_t) \right] \\

p(z_t \mid x_t)
=

\det(2\pi Q_t)^{-\frac{1}{2}}
\exp
\left[
-\frac{1}{2}
(z_t-C_tx_t)^TQ_t^{-1}(z_t-C_tx_t)
\right]
$$


1. prediction step
2. correction step

---

Kalman filter algorithm

input: $\mu_{t-1}, \Sigma_{t-1}, u_t, z_t$ (average, covariance, input, observation)
$$
\bar\mu_t=A_t \mu_{t-1}+B_t u_t \\
\bar \Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t \\
\\
K_t = \bar \Sigma_t C^T_t (C_t \bar\Sigma_t C_t^T +Q_t )^{-1} \\
\mu_t=\bar\mu_t+K_t(z_t-C_t\bar\mu_t) \\
\Sigma_t = (I-K_t C_t)\bar\Sigma_t
$$
output: $\mu_t, \Sigma_t$ 

### explain

> [!WARNING]
>
> more proof is need



The Kalman filter algorithm has exactly that form because your two Bayes-filter steps preserve a **Gaussian belief** when both the motion model and measurement model are linear-Gaussian.

You start with the Bayes filter:

$$
\overline{bel}(x_t)=
\int p(x_t|x_{t-1},u_t),bel(x_{t-1}),dx_{t-1}
$$

$$
bel(x_t)=\eta,p(z_t|x_t),\overline{bel}(x_t)
$$

Kalman filter is just the closed-form solution when all distributions are Gaussian.

---

Assume the previous belief is Gaussian:

$$
bel(x_{t-1})=\mathcal N(\mu_{t-1},\Sigma_{t-1})
$$

and your motion model is

$$
x_t = A_t x_{t-1}+B_tu_t+\epsilon_t,
\qquad
\epsilon_t\sim \mathcal N(0,R_t)
$$

This is the same as your density:

$$
p(x_t|u_t,x_{t-1})=\mathcal N(A_tx_{t-1}+B_tu_t,;R_t)
$$

#### prediction

Now ask: if $x_{t-1}$ is Gaussian, what is the distribution of
$$
x_t=A_tx_{t-1}+B_tu_t+\epsilon_t?
$$

A linear transformation of a Gaussian is still Gaussian. So the predicted belief is also Gaussian:

$$
\overline{bel}(x_t)=\mathcal N(\bar\mu_t,\bar\Sigma_t)
$$

with

$$
\bar\mu_t=A_t\mu_{t-1}+B_tu_t
$$

and

by covariance after linear transformation

> [A geometric interpretation of the covariance matrix](https://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/)

$$
\bar\Sigma_t=A_t\Sigma_{t-1}A_t^T+R_t
$$

That explains lines 2 and 3 of the algorithm.

Intuition:

$$
A_t\Sigma_{t-1}A_t^T
$$

is the old uncertainty transformed through the dynamics, and

$$
R_t
$$

is extra uncertainty from motion noise.

#### correction

Then comes the correction step:
$$
bel(x_t)=\eta p(z_t|x_t)\overline{bel}(x_t)
$$

And
$$
z_t=C_tx_t+\delta_t,
\qquad
\delta_t\sim\mathcal N(0,Q_t)
$$


So measurement model is in general form, 
$$
p(z_t|x_t)=\mathcal N(C_tx_t,Q_t)
$$

Now we multiply two Gaussians:

$$
bel(x_t)\propto
\mathcal N(x_t;\bar\mu_t,\bar\Sigma_t)
\mathcal N(z_t;C_tx_t,Q_t)
$$

The result is still Gaussian:

$$
bel(x_t)=\mathcal N(\mu_t,\Sigma_t)
$$

The updated mean is

$$
\mu_t=\bar\mu_t+K_t(z_t-C_t\bar\mu_t)
$$

This is line 5.

The term

$$
z_t-C_t\bar\mu_t
$$

is called the **innovation** or **residual**. It means:

$$
\text{actual measurement} - \text{predicted measurement}
$$

So the Kalman update says:

$$
\text{new estimate}
=

\text{prediction}
+
\text{correction}
$$

where the correction is proportional to how wrong the predicted measurement was.

---

The matrix

$$
K_t
$$

is the **Kalman gain**:

$$
K_t=\bar\Sigma_t C_t^T(C_t\bar\Sigma_t C_t^T+Q_t)^{-1}
$$

It decides how much to trust the sensor versus the prediction.

If measurement noise $Q_t$ is large, then

$$
K_t
$$

gets smaller, so the filter trusts the prediction more.

If prediction uncertainty $\bar\Sigma_t$ is large, then

$$
K_t
$$

gets larger, so the filter trusts the measurement more.

The denominator

$$
C_t\bar\Sigma_tC_t^T+Q_t
$$

is the predicted uncertainty in the measurement space. It includes uncertainty from the predicted state plus measurement noise.

---

The covariance update is

$$
\Sigma_t=(I-K_tC_t)\bar\Sigma_t
$$

This is line 6.

Intuitively, after seeing a measurement, uncertainty should decrease. The term

$$
K_tC_t
$$

represents how much information the measurement gives about the state. So

$$
I-K_tC_t
$$

shrinks the predicted covariance.



So the algorithm is basically:

1. **Predict state mean**

$$
\bar\mu_t=A_t\mu_{t-1}+B_tu_t
$$

2. **Predict state uncertainty**

$$
\bar\Sigma_t=A_t\Sigma_{t-1}A_t^T+R_t
$$

3. **Compute how much to trust measurement**

$$
K_t=\bar\Sigma_t C_t^T(C_t\bar\Sigma_t C_t^T+Q_t)^{-1}
$$

4. **Correct mean using measurement error**

$$
\mu_t=\bar\mu_t+K_t(z_t-C_t\bar\mu_t)
$$

5. **Reduce uncertainty**

$$
\Sigma_t=(I-K_tC_t)\bar\Sigma_t
$$

That is why the Kalman filter has this form: it is the Bayes filter specialized to **linear dynamics + Gaussian noise + Gaussian belief**.

