package org.team100.lib.uncertainty;

/**
 * Methods governing vision update uncertainties.
 * 
 * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
 * https://docs.google.com/spreadsheets/d/1StMbOyksydpzFmpHbZBL7ICMQtHnOBubrOMeYSx0M6E
 */
public class Uncertainty {
    /**
     * Standard deviation of vision updates in SE(2).
     * 
     * Data comes from eyeballing the graphs in the Wang paper.
     * 
     * The vision system is good at estimating range from the tag, but it is not
     * very good at estimating bearing from the tag. The large uncertainty in
     * bearing results in a crescent-shaped uncertainty in the robot position. We
     * don't have a way to represent that shape, so instead we just take the
     * maximum.
     * 
     * Note: due to the uncertainty singularity on the tag axis, when we are
     * directly in front of the tag, we can't use it at all.
     */
    public static IsotropicNoiseSE2 visionMeasurementStdDevs(double distanceM, double offAxisAngleRad) {
        if (distanceM < 0)
            throw new IllegalArgumentException();
        if (offAxisAngleRad < 0)
            throw new IllegalArgumentException();
        // these extra 0.01 values are total guesses
        // TODO: calibrate this, remove it?
        double cartesianErrorM = figure5(distanceM) + 0.01;
        double rotationErrorRad = figure6(offAxisAngleRad) + 0.01;
        double rotationEffectM = distanceM * rotationErrorRad;
        double maxCartesian = Math.max(cartesianErrorM, rotationEffectM);
        return IsotropicNoiseSE2.fromStdDev(maxCartesian, rotationErrorRad);
    }

    /**
     * Figure 5 in the Wang paper (below 15 m) indicates a linear relationship
     * between cartesian error and tag distance.
     *
     * TODO: calibrate this
     */
    static double figure5(double distanceM) {
        return 0.03 * distanceM;
    }

    /**
     * Figure 6 in the Wang paper indicates a U-shaped relationship between the "off
     * axis" angle and the error. Note the infinite error at zero.
     * 
     * Remember that our tag normal direction is "into the page", but the "off axis"
     * normal is "out of the page".
     * 
     * TODO: calibrate this
     */
    static double figure6(double offAxisAngleRad) {
        if (offAxisAngleRad < 0)
            throw new IllegalArgumentException("angle must be non-negative");
        if (offAxisAngleRad > 1.5) {
            // tag from behind makes no sense
            return 0;
        }
        // This uses degrees because figure 6 uses degrees.
        double offAxisDegrees = Math.toDegrees(offAxisAngleRad);
        if (offAxisDegrees < 3)
            // return Double.MAX_VALUE;
            // avoid infinity so that downstream can tone it down
            return 3;
        double errorDeg = 10 / offAxisDegrees + 10 / Math.pow(85 - offAxisDegrees, 1.2);
        return Math.toRadians(errorDeg);
    }

    /**
     * The error in odometry is superlinear in speed. Since the odometry samples
     * happen regularly, we can use the sample distance as a measure of speed.
     * 
     * This yields zero when the robot isn't moving, which is what you'd expect.
     * 
     * I completely made this up
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=995645441#gid=995645441
     */
    public static double odometryCartesianStdDev(double distanceM) {
        double norm = Math.abs(distanceM);
        // We kinda measured 5% error in the best (slow) case, in 2024.
        double lowSpeedError = 0.05;
        // This is just a guess
        double superError = 0.5;
        return lowSpeedError * norm + superError * norm * norm;
    }

    /**
     * How does rotation error scale with speed? Driving in a straight line
     * definitely produces rotational drift, so this isn't just proportional to the
     * odometry rotation term alone.
     * 
     * Maybe just add them, 1 meter == 1 radian.
     * 
     * TODO: measure this for real.
     */
    public static double odometryRotationStdDev(double distanceM, double rotationRad) {
        double norm = Math.abs(distanceM) + Math.abs(rotationRad);
        // We kinda measured 5% error in the best (slow) case.
        double lowSpeedError = 0.05;
        // This is just a guess
        double superError = 0.5;
        // We haven't measured this, so just guess it's the same???
        return lowSpeedError * norm + superError * norm * norm;
    }
}
