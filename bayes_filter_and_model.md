

# Bayes filter and model

No map at this point

## state estimation

**probability distribution** over possible positions

Goal

Estimate the state $x$ of a system given observations $z$ and controls $u$ 
$$
p(x|z,u)
$$

## get recursive Bayes filter form 

Definition of the belief
$$
bel(x_t)=p(x_t|z_{1:t},u_{1:t})\\
=p(x_t|z_1,z_2...z_t,u_1,u_2...u_t)\\
=p(x_t|z_t,z_{1:t-1},u_{1:t})
$$
by bayes' rule

>
>$$
>P(A|B)=\frac{P(B|A)P(A)}{P(B)}\\
>=\eta \cdot P(B|A) P(A)
>$$
>
>* $\eta=\frac{1}{P(B)}$ is normalizing term
>* $P(B|A)$ is the **likelihood** of the data given the parameters.
>* $P(A)$ is the **prior belief** about the parameters.
>
>
>
>conditional version of Bayes' rule is:
>$$
>P(A|B,C)=\frac{P(B|A,C)P(A|C)}{P(B|C)}\\
>=\eta \cdot P(B|A,C) P(A|C)
>$$
>

$A=x_t$, $B=z_{t}$, condition on all previous information $C=z_{1:t-1},u_{1:t}$.
$$
=\eta p(z_t|x_t,z_{1:t-1},u_{1:t}) p(x_t|z_{1:t-1},u_{1:t})
$$


* measurement likelihood: $p(z_t|x_t,z_{1:t-1},u_{1:t})$​
  * If the robot were at state $x_t$, how likely would it be to observe measurement $z_t$​?
  * If the robot thinks it is near a wall, and the LiDAR sees a wall nearby, then this probability is high.
* prior belief: $p(x_t|z_{1:t-1}, u_{1:t})$​ 
  * This is the predicted state before using the newest sensor measurement.
    * This is the prediction before seeing the new measurement.
  * Given the previous pose and the robot’s motion command, where do we expect the robot to be now?

by Markov assumption: that the probability of a future event depends solely on the current  state, completely independent of the historical path taken to get there
$$
=\eta p(z_t|x_t) p(x_t|z_{1:t-1},u_{1:t})
$$
by law of total probability

> For continuous variables
> $$
> P(A)=\int P(A|B)P(B) dB
> $$
>
> * $p(B)$: probability density function
>
> 
>
> With other condition variables $C$
> $$
> P(A|C)=\int P(A|B,C)P(B|C) dB
> $$

* $A=x_t$
* $B=x_{t-1}$
* $C=z_{1:t-1},u_{1:t}$

$$
=\eta p(z_t|x_t)\int p(x_t|x_{t-1},z_{1:t-1},u_{1:t})p(x_{t-1}|z_{1:t-1},u_{1:t})dx_{t-1}
$$

* $P(A|B,C)$
  * If the previous state was $x_{t-1}$, how likely is the current state $x_t$?
* $P(B|C)$
  * How likely was that previous state $x_{t-1}$, given the information so far?

and again by markov assumption
$$
=\eta p(z_t|x_t)\int p(x_t|x_{t-1},u_{t})p(x_{t-1}|z_{1:t-1},u_{1:t})dx_{t-1}
$$
and because $u_t$ is the **control applied after time $t-1$**. it does **not** affect the already-existing previous state $x_{t-1}$.
$$
=\eta p(z_t|x_t)\int p(x_t|x_{t-1},u_{t})p(x_{t-1}|z_{1:t-1},u_{1:t-1})dx_{t-1}\\
=\eta p(z_t|x_t)\int p(x_t|x_{t-1},u_{t}) bel(x_{t-1}) dx_{t-1}
$$
this is recursive form we want

### explain more on recursive form

we can further explain/view recursive from as 2 steps

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



## Probabilistic Motion Models

Specifies a posterior probability that action $u$ carries the robot from $x_{t-1}$ to $x_t$.
$$
p(x_t|x_{t-1},u_{t})
$$
In practice, one often finds two types of motion models:

* odometry-based
  * usually used when wheel encoders available
* velocity-based
  * used when wheel encoders is unavailable

### odometry model

**odometry information is classified as the control input**:

Given the previous state $x_{t-1}$ and the odometry/control input $u_t$, what is the probability of the new state $x_t$?

For 2D plane
$$
u_t=(\delta_{rot1}, \delta_{trans}, \delta_{rot2})
$$


### velocity model

$$
u=(v,w)^T
$$

linear velocity concat angular velocity

the new angle is 
$$
\theta_{t+1}=\theta_{t}+\omega \Delta
$$


to model noise (uncertainty),
$$
\theta_{t+1}=\theta_{t}+\omega \Delta t+\gamma \Delta t
$$


## sensor model

recall correction steps
$$
bel(x_t)=\eta p(z_t|x_t)\overline{bel}(x_t)
$$
and sensor model
$$
p(z_t|x_t)
$$

### laser scanner

Scan $z$ consists of $K$ measurements
$$
z_t=\{z_t^1, ..., z_t^k\}
$$
And since individual measurements are independent given the robot position, sensor model can therefore model as
$$
p(z_t|x_t,m)=\prod_{i=1}^{k}p(z_t^i | x_t,m)
$$

### Model for Perceiving Landmarks with Range-Bearing Sensors

* range bearing: $z_t^i=(r_t^i, \phi_t^i)^T$
* robot's pose $(x,y,\theta)^T$ 
* observation of feature $j$ at location $(m_{j,x},m_{j,y})^T$
  * $Q_t$ is uncertainty

$$
\begin{bmatrix}
r_t^i \\
\phi_t^i
\end{bmatrix}=
\begin{bmatrix}
\sqrt{(m_{j,x}-x)^2+(m_{j,y}-y)^2} \\
atan2(m_{j,y}-y, m_{j,x}-x)-\theta
\end{bmatrix}
+Q_t
$$

## ref

* [ 統一的框架 Bayes Filter ](https://bobondemon.github.io/2017/05/10/Bayes-Filter-for-Localization/)
* [[SLAM] 學習筆記 03— Bayes Filter](https://medium.com/@tiffanychen1118/slam-%E5%AD%B8%E7%BF%92%E7%AD%86%E8%A8%98-03-bayes-filter-a4c9874bc067)
* 