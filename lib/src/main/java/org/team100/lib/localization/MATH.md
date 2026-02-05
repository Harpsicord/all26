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
* Odometry variance adds to state variance.
* Gyro variance adds to state variance.
* Gyro input is moderated by odometry, dependent on speed.
* The main rotation input is now vision, not the gyro.
* The gyro cannot be reset: use a tag for that.

So there are two types of updates.

Each odometry update is simply addition, where we assume that odometry
measurement and state are independent, so the means add, and the variances add.
The assumption of independence comes from the assumption of independence
of each odometry update.  In reality, the main odometry error is probably
bias rather than noise; we ignore that.

For convenience, gyro updates are combined with odometry updates prior to
the state update, using another layer of inverse-variance pooling.  The
odometry uncertainty is low while moving slowly, and higher while moving
fast.

Each vision update is a weighted average, using inverse variance weighting,
or, equivalently, covariance intersection with equal weights.

```math
\mu = \left(
\frac{1}{\sigma_A^2} + \frac{1}{\sigma_B^2}
\right)^{-1}
\left(
\frac{\mu_A}{\sigma_A^2}
+
\frac{\mu_B}{\sigma_B^2}
\right)

```


The variance of the fused result is a function of the component variances,
and also the difference in means:

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


References:

* A useful question and describing the covariance due to dispersion of the mean [Stack Overflow](https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians).
* The [sway](https://github.com/Team100/all24/blob/main/studies/sway/src/main/java/org/team100/lib/sway/fusion/LinearPooling.java#L149) project from 2023, which was an unused attempt to make all observations into random variables.
* A [survey of pooling methods](https://arxiv.org/pdf/2202.11633)
* Wikipedia on [inverse variance weighting](https://en.wikipedia.org/wiki/Inverse-variance_weighting).
* Wikipedia on [covariance intersection](https://en.wikipedia.org/wiki/Covariance_intersection), which is the same, if the weights are chosen to be equal.
