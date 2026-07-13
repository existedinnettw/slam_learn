# Grid map

Grid maps are a map representation for SLAM and mapping with known poses.
Instead of detecting and tracking landmarks, the environment is discretized
into small cells.

Each cell answers one binary question:

$$
\text{Is this part of space occupied?}
$$

This makes grid maps useful when the environment does not contain reliable
predefined features, or when the robot needs a dense representation for
navigation and obstacle avoidance.

## Feature maps vs grid maps

Feature maps represent the world by landmarks:

* points
* lines
* corners
* trees, poles, doors, walls, etc.

This is compact and works naturally with Kalman-filter-based SLAM. Repeated
observations of the same feature improve the feature position estimate.

Grid maps represent the world by cells:

* the world is split into a regular grid
* each cell is occupied or free
* the map does not require a feature detector
* the model is non-parametric
* large maps can require substantial memory

The main tradeoff:

| Representation | Strength | Weakness |
| --- | --- | --- |
| Feature map | Compact, good for landmark-based SLAM | Needs reliable feature detection |
| Grid map | Dense, simple, no feature detector | Memory grows with map size and resolution |

## Occupancy grid assumption

The first assumption is that the area covered by one cell is either completely
free or completely occupied.

This is an approximation. A real cell can contain both free and occupied
space, especially if the resolution is coarse. The approximation becomes more
accurate when the grid resolution is fine.

For cell $m_i$:

$$
m_i =
\begin{cases}
1 & \text{occupied} \\
0 & \text{free}
\end{cases}
$$

Each cell is a binary random variable:

$$
p(m_i = 1)
$$

or simply:

$$
p(m_i)
$$

If

$$
p(m_i)=0.5
$$

then the map has no information about that cell. Occupied and free are equally
likely.

## Static world assumption

Most occupancy-grid mapping systems assume that the world is static:

* a wall cell stays occupied
* an empty-space cell stays free
* the map does not explicitly model moving objects

This assumption is not always true in practice. People, doors, chairs, and
cars can move. Standard occupancy-grid mapping treats those as sensor noise or
inconsistent observations unless extra dynamic-map logic is added.

## Cell independence assumption

The cells are assumed to be independent random variables.

This means that knowing whether one cell is occupied does not directly change
the probability of another cell:

$$
p(m)=\prod_i p(m_i)
$$

where:

* $m$ is the whole map
* $m_i$ is one grid cell

This is a strong approximation. In the real world, neighboring cells are often
correlated. For example, if one cell is part of a wall, nearby cells are likely
to be wall too. The independence assumption is used because it makes the map
easy to update: every cell can be estimated separately.

## Mapping with known poses

Occupancy grid mapping is often described as **mapping with known poses**.

Given:

* sensor measurements $z_{1:t}$
* known robot poses $x_{1:t}$

Estimate:

* the map $m$

Formally:

$$
p(m \mid z_{1:t},x_{1:t})
$$

Using the cell independence assumption:

$$
p(m \mid z_{1:t},x_{1:t})
=
\prod_i p(m_i \mid z_{1:t},x_{1:t})
$$

So the full map problem becomes many small binary estimation problems, one for
each cell.

## Static state binary Bayes filter

For one cell $m_i$, the state is static because the cell does not move. The
Bayes update is:

$$
p(m_i \mid z_{1:t},x_{1:t})
=
\eta
p(z_t \mid m_i,x_t)
p(m_i \mid z_{1:t-1},x_{1:t-1})
$$

where:

* $p(m_i \mid z_{1:t-1},x_{1:t-1})$ is the previous belief
* $p(z_t \mid m_i,x_t)$ is the sensor likelihood
* $\eta$ normalizes the result

For the opposite event, the cell is free:

$$
p(\neg m_i \mid z_{1:t},x_{1:t})
=
\eta
p(z_t \mid \neg m_i,x_t)
p(\neg m_i \mid z_{1:t-1},x_{1:t-1})
$$

Taking the ratio cancels the normalizer:

$$
\frac{
p(m_i \mid z_{1:t},x_{1:t})
}{
p(\neg m_i \mid z_{1:t},x_{1:t})
}
=
\frac{
p(z_t \mid m_i,x_t)
}{
p(z_t \mid \neg m_i,x_t)
}
\frac{
p(m_i \mid z_{1:t-1},x_{1:t-1})
}{
p(\neg m_i \mid z_{1:t-1},x_{1:t-1})
}
$$

This ratio is the odds that the cell is occupied.

## Odds and probability

The odds form is:

$$
odds(m_i)=\frac{p(m_i)}{1-p(m_i)}
$$

Recover the probability from odds:

$$
p(m_i)=\frac{odds(m_i)}{1+odds(m_i)}
$$

Updating odds directly is possible, but multiplication is less convenient than
addition. Occupancy-grid mapping therefore usually uses log odds.

## Log odds notation

The log odds of cell $m_i$ is:

$$
l_i =
\log
\frac{p(m_i)}{1-p(m_i)}
$$

The inverse conversion is:

$$
p(m_i)
=
1-\frac{1}{1+\exp(l_i)}
=
\frac{1}{1+\exp(-l_i)}
$$

Useful values:

| Probability | Log odds | Meaning |
| --- | --- | --- |
| $p(m_i)=0.5$ | $l_i=0$ | unknown |
| $p(m_i)>0.5$ | $l_i>0$ | likely occupied |
| $p(m_i)<0.5$ | $l_i<0$ | likely free |

## Occupancy mapping in log odds form

Let:

$$
l_{t,i}
=
\log
\frac{
p(m_i \mid z_{1:t},x_{1:t})
}{
1-p(m_i \mid z_{1:t},x_{1:t})
}
$$

The recursive update is:

$$
l_{t,i}
=
l_{t-1,i}
+
\log
\frac{
p(m_i \mid z_t,x_t)
}{
1-p(m_i \mid z_t,x_t)
}
-
l_{0,i}
$$

where:

* $l_{t,i}$ is the current log odds of cell $i$
* $l_{t-1,i}$ is the previous log odds
* $p(m_i \mid z_t,x_t)$ comes from the inverse sensor model
* $l_{0,i}$ is the prior log odds

If the prior is unknown:

$$
p(m_i)=0.5
\quad\Rightarrow\quad
l_{0,i}=0
$$

Then the update becomes:

$$
l_{t,i}
=
l_{t-1,i}
+
\operatorname{inverseSensorModel}(m_i,z_t,x_t)
$$

This is why log odds is efficient: each update is just addition.

## Occupancy grid mapping algorithm

For every measurement $z_t$:

1. Use the known pose $x_t$ to project the sensor beam into the map.
2. Find the cells affected by the measurement.
3. For each affected cell, compute the inverse sensor model.
4. Add the log-odds update to that cell.
5. Convert log odds back to probability when the map must be displayed or used.

Pseudo-code:

```text
for each cell m_i:
    if m_i is affected by measurement z_t:
        l_i = l_i + inverse_sensor_model(m_i, z_t, x_t) - l0
```

In practice, values are often clipped:

$$
l_{\min} \le l_i \le l_{\max}
$$

Clipping prevents a cell from becoming impossible to change after many repeated
observations.

## Inverse sensor model

The inverse sensor model answers:

> Given the current measurement and pose, what should this cell believe?

That is:

$$
p(m_i \mid z_t,x_t)
$$

This is different from the forward sensor model:

$$
p(z_t \mid m_i,x_t)
$$

The forward model predicts a measurement from a known map. The inverse model
updates the map from a measurement.

## Sonar range sensor intuition

For a range sensor measurement $z$:

* cells before the measured obstacle are likely free
* cells near the measured distance are likely occupied
* cells far behind the measured distance receive no information

For a cell at distance $r$ along the sensor axis:

| Cell location | Update |
| --- | --- |
| $r < z$ | free evidence |
| $r \approx z$ | occupied evidence |
| $r > z$ | no information |

Sonar sensors are noisy and wide-beam, so the inverse model usually spreads
evidence over a cone instead of a thin line.

## Laser range finder intuition

Laser range finders are more precise than sonar. A common approximation is to
treat each laser beam as a line:

* cells crossed before the endpoint are free
* the endpoint cell is occupied
* cells behind the endpoint are unknown

This makes laser occupancy-grid mapping fast and effective.

For a beam with measured range $z$:

$$
\operatorname{update}(m_i)=
\begin{cases}
l_{\text{free}} & \text{cell lies before the measured endpoint} \\
l_{\text{occ}} & \text{cell lies near the measured endpoint} \\
0 & \text{cell is not affected by the beam}
\end{cases}
$$

where:

$$
l_{\text{free}} < 0,
\qquad
l_{\text{occ}} > 0
$$

## Maximum likelihood grid maps

Another view is to estimate each cell value by maximum likelihood.

For a binary cell, each observation is a Bernoulli trial. Let:

* $n_i^{occ}$ be the number of observations that say cell $i$ is occupied
* $n_i^{free}$ be the number of observations that say cell $i$ is free

The maximum likelihood estimate is:

$$
\hat p_i
=
\frac{n_i^{occ}}{n_i^{occ}+n_i^{free}}
$$

This is simple, but it ignores prior belief. For cells with only a few
observations, that can be too confident.

## Posterior distribution of a cell

A Bernoulli likelihood has a Beta conjugate prior:

$$
p_i \sim \operatorname{Beta}(\alpha,\beta)
$$

After observing:

* $n_i^{occ}$ occupied observations
* $n_i^{free}$ free observations

the posterior is:

$$
p_i \mid z_{1:t}
\sim
\operatorname{Beta}
(\alpha+n_i^{occ},\beta+n_i^{free})
$$

The expected value is:

$$
\mathbb E[p_i]
=
\frac{\alpha+n_i^{occ}}
{\alpha+\beta+n_i^{occ}+n_i^{free}}
$$

The maximum a posteriori estimate, when the parameters are greater than one,
is:

$$
p_i^{MAP}
=
\frac{\alpha+n_i^{occ}-1}
{\alpha+\beta+n_i^{occ}+n_i^{free}-2}
$$

With a uniform prior:

$$
\operatorname{Beta}(1,1)
$$

the MAP estimate becomes the same as the maximum likelihood estimate:

$$
p_i^{MAP}
=
\frac{n_i^{occ}}{n_i^{occ}+n_i^{free}}
$$

## Summary

Occupancy grid maps:

* discretize space into independent cells
* model each cell as a binary random variable
* use static-state binary Bayes filters per cell
* work well for mapping with known poses
* update efficiently in log odds form
* do not require predefined features

The key practical formula is:

$$
l_{t,i}
=
l_{t-1,i}
+
\log
\frac{
p(m_i \mid z_t,x_t)
}{
1-p(m_i \mid z_t,x_t)
}
-
l_{0,i}
$$

For an unknown prior, $l_{0,i}=0$, so mapping becomes repeated addition of
inverse-sensor-model evidence.

## Ref

* `out/slam10-gridmaps.pdf`
* Thrun, Burgard, and Fox, *Probabilistic Robotics*, Chapter 4.2 and Chapter
  9.1-9.2
