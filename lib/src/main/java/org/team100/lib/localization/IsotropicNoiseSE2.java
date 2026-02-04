package org.team100.lib.localization;

/**
 * Represents measurement and estimate uncertainty in the situation we actually
 * have, where x and y are equivalent in every respect, and the covariances in
 * SE(2) are assumed to be zero for simplicity. This leaves only two numbers.
 */
public class IsotropicNoiseSE2 {
    // we store variance to avoid all those squares and square roots.
    private final double m_cartesianVariance;
    private final double m_rotationVariance;

    /**
     * @param cartesianStdDev Standard deviation of cartesian dimensions.
     * @param rotationStdDev  Standard deviation of rotation.
     */
    IsotropicNoiseSE2(double cartesianStdDev, double rotationStdDev) {
        m_cartesianVariance = Math.pow(cartesianStdDev, 2);
        m_rotationVariance = Math.pow(rotationStdDev, 2);
    }

    public double cartesianVariance() {
        return m_cartesianVariance;
    }

    public double rotationVariance() {
        return m_rotationVariance;
    }

    /** For testing only. */
    double cartesian() {
        return Math.sqrt(m_cartesianVariance);
    }

    /** For testing only. */
    double rotation() {
        return Math.sqrt(m_rotationVariance);
    }

}
