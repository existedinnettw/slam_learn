# SLAM introduction

SLAM = localization  + mapping

 Given

* robot controls 
  * $u_{1:T}=\{u_1, u_2, ..., u_T\}$

* Observationns
  * $z_{1:T}=\{z_1, z_2, ..., z_T\}$

Wanted

* **map** of the environment
  * $m$
* **path** of the robot
  * $x_{0:T}=\{x_0, x_1, x_2,..., x_T\}$ 



## formulization


It is a **multiple integral over the previous robot poses**:

Here, the integrals are over the **state variables** $x_0, x_1, \dots, x_{t-1}$. So it is a **marginalization integral**.

The recursive version is the important part. Instead of doing:

$$
p(x_t,m \mid z_{1:t},u_{1:t})
=

\int \cdots \int p(x_{0:t},m \mid z_{1:t},u_{1:t}),dx_{t-1}\cdots dx_0
$$

all at once, online SLAM updates one time step at a time.

### example


This means:

> To get the probability of the **current pose** $x_t$ and map $m$, add up all possibilities for the old poses $x_0,\dots,x_{t-1}$.

It is the continuous version of “summing out” variables.

For example, if $t=3$, then:

$$
p(x_3,m \mid z_{1:3},u_{1:3})
=

\int \int \int
p(x_0,x_1,x_2,x_3,m \mid z_{1:3},u_{1:3})
, dx_2,dx_1,dx_0
$$

You are saying:

$$
\text{current belief}
=

\text{all histories that end at } x_3
$$

So you integrate over all possible previous paths:

$$
x_0 \rightarrow x_1 \rightarrow x_2 \rightarrow x_3
$$

because you no longer care exactly which old path happened.

Now the recursive part comes from doing the integrals one at a time.

For $t=3$:

$$
p(x_3,m \mid z_{1:3},u_{1:3})
=

\int
\left[
\int
\left[
\int
p(x_0,x_1,x_2,x_3,m \mid z_{1:3},u_{1:3})
, dx_0
\right]
dx_1
\right]
dx_2
$$

So instead of one huge operation, you can do:

First remove (x_0):

$$
p(x_1,x_2,x_3,m \mid z_{1:3},u_{1:3})
=

\int
p(x_0,x_1,x_2,x_3,m \mid z_{1:3},u_{1:3})
, dx_0
$$

Then remove $x_1$:

$$
p(x_2,x_3,m \mid z_{1:3},u_{1:3})
=

\int
p(x_1,x_2,x_3,m \mid z_{1:3},u_{1:3})
, dx_1
$$

Then remove $x_2$:

$$
p(x_3,m \mid z_{1:3},u_{1:3})
=

\int
p(x_2,x_3,m \mid z_{1:3},u_{1:3})
, dx_2
$$

That is why it is recursive: each step removes **one older pose**.

The intuition is similar to this discrete example.



by **Fubini's theorem**: the ability to change the integration order.

$dx_0...dx_{t-1}$ become $dx_{t-1}...dx_0$ 

## online SLAM

> ![WARNING] This is advanced, you should skip until bayes filter model and map is introduced.

<!-- TODO: add explanation -->

## localization

* visual
  - use landmarks to estimate robot location
  - camera, lidar
* odemetry
  - use origin starting location to estimate robot location
