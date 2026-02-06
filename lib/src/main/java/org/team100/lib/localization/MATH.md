# Using Uncertainty

In 2026, our alignment requirements are tighter than in previous
years, so we need to pay more attention to localization accuracy.

Previously, we made several simplifying choices:

* A fixed estimate for state variance.
* The variance of vision-derived XY position depended on tag distance.
* The variance of vision-derived rotation was infinite.
* Odometry was assumed to be perfect, i.e. did not change state variance.
* The gyro was assumed to be perfect: other rotation sources were ignored.
* The gyro could be reset manually.

In 2026, most of these choices are replaced:

* State variance evolves as it is updated.
* Vision-derived XY position variance still depends on tag distance.
* Vision-derived rotation variance depends on off-axis angle.
* Odometry and odometry variances add to state variance.
* Gyro variance is constant.
* Odometry variance is dependent on speed.
* The only absolute rotation input is now vision, not the gyro.
* Therefore the gyro cannot be reset: show a tag to a camera to set the rotation.

So there are two types of updates: "between" updates,
which describe the difference between the previous state and the next one,
and "fusion" updates, which are independent estimates of the state itself.

Each "between" update is itself a fusion of two independent estimates,
one from odometry, and one from the gyro.

To apply the "between" update to the state,  we assume that update
measurement and state are independent, so the means add, and the variances add,
with no covariance term.
The assumption of independence comes from the assumption of independence
of each odometry update.  In reality, the main odometry error is probably
bias rather than noise, but we ignore that.

Each "fusion" update computes the resulting state mean and variance using
inverse-variance weighting, or, equivalently, covariance intersection
with equal weights.

To fuse the gyro and odometry, the gyro variance is a constant, larger than
the drift rate, and the odometry variance is quadratic with respect to speed.

To fuse the vision input, variances are drawn from the Wang paper,
and the state variance is whatever has accumulated.

The mean of each fused result is as follows, straightforward weighting by
inverse variance:

```math
\mu = w_A \mu_A + w_B \mu_B
```

The variance of each fused result is a function of the component variances,
with a covariance term from the difference in means:

```math
\sigma^2=w_A\sigma_A^2+w_B\sigma_B^2+w_A w_B(\mu_A-\mu_B)^2
```

The weights are determined using inverse-variance weighting.

```math
w_A = \frac{1}{\sigma_A^2}\left(  \frac{1}{\sigma_A^2} + \frac{1}{\sigma_B^2} \right)^{-1}
```
and
```math
w_B = \frac{1}{\sigma_B^2}\left(  \frac{1}{\sigma_A^2} + \frac{1}{\sigma_B^2} \right)^{-1}

```

Note: the gyro produces both an estimate of position, and an estimate of
velocity.  The robot state also contains both postion and velocity.  We could
separate these two dimensions, so that the gyro velocity would update the
state velocity independently of the position update, but we're not doing that.
The gyro position is a simple integral of the velocity, and the state velocity
is a simple difference of the position, so we just ignore the gyro velocity.


References:

* A useful question and describing the covariance due to dispersion of the mean [Stack Overflow](https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians).
* The [sway](https://github.com/Team100/all24/blob/main/studies/sway/src/main/java/org/team100/lib/sway/fusion/LinearPooling.java#L149) project from 2023, which was an unused attempt to make all observations into random variables.
* A [survey of pooling methods](https://arxiv.org/pdf/2202.11633)
* Wikipedia on [inverse variance weighting](https://en.wikipedia.org/wiki/Inverse-variance_weighting).
* Wikipedia on [covariance intersection](https://en.wikipedia.org/wiki/Covariance_intersection), which is the same, if the weights are chosen to be equal.
