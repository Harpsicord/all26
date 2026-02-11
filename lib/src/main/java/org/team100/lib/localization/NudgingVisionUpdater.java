package org.team100.lib.localization;

import org.team100.lib.coherence.Takt;
import org.team100.lib.fusion.CovarianceInflation;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Updates SwerveModelHistory with any vision input, by interpolating to find a
 * pose for the vision timestamp, nudging that pose towards the vision
 * measurement, and then asking the odometry updater to replay all the later
 * odometry.
 * 
 * The "nudging" here is essentially just a weighted average; you provide the
 * weights you want at update time.
 */
public class NudgingVisionUpdater implements VisionUpdater {

    private final SwerveHistory m_history;
    /** For replay. */
    private final OdometryUpdater m_odometryUpdater;
    /** To measure time since last update, for indicator. */
    private double m_latestTimeS;

    public NudgingVisionUpdater(
            SwerveHistory history,
            OdometryUpdater odometryUpdater) {
        m_history = history;
        m_odometryUpdater = odometryUpdater;
        m_latestTimeS = 0;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     * 
     * @param timestamp   When the measurement was made.
     * @param measurement Robot pose from vision.
     * @param visionNoise Measurement noise in SE(2).
     */
    @Override
    public void put(
            double timestamp,
            Pose2d measurement,
            IsotropicNoiseSE2 visionNoise) {
        // System.out.printf("vision updater visionNoise %s\n", visionNoise);

        // Skip too-old measurement
        if (m_history.tooOld(timestamp)) {
            return;
        }

        // Sample the history at the measurement time.
        SwerveState sample = m_history.getRecord(timestamp);

        // Nudge the sample pose towards the measurement.
        ModelSE2 sampleModel = sample.state();
        Pose2d samplePose = sampleModel.pose();
        // Vision updates to not affect the velocity estimate.
        VelocitySE2 sampleVelocity = sampleModel.velocity();
        IsotropicNoiseSE2 sampleNoise = sample.noise();
        SwerveModulePositions samplePositions = sample.positions();
        // Vision updates do not affect the gyro measurement.
        Rotation2d sampleGyroYaw = sample.gyroYaw();
        // Vision updates do not affect the gyro bias estimate.
        VariableR1 sampleGyroBias = sample.gyroBias();

        NoisyPose2d nudged = nudge(
                new NoisyPose2d(samplePose, sampleNoise),
                new NoisyPose2d(measurement, visionNoise));

        ModelSE2 model = new ModelSE2(nudged.pose(), sampleVelocity);

        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.inverseVarianceWeightedAverage(
                sampleNoise, visionNoise);

        // System.out.printf("vision put noise %s\n", noise);

        // Remember the result.
        m_history.put(timestamp, model, noise, samplePositions, sampleGyroYaw, sampleGyroBias);

        // Replay everything after the sample.
        m_odometryUpdater.replay(timestamp);

        // Remember the time of this update.
        m_latestTimeS = Takt.get();
    }

    /**
     * The age of the last pose estimate, in seconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        return Takt.get() - m_latestTimeS;
    }

    /////////////////////////////////////////

    /**
     * Compute the weighted average of sample and measurement, using
     * inverse-variance weighting.
     * 
     * The variance of the cartesian part is assumed to be isotropic.
     * 
     * @param sample      historical pose
     * @param measurement new (vision) input
     */
    static NoisyPose2d nudge(
            NoisyPose2d noisySample,
            NoisyPose2d noisyMeasurement) {

        Pose2d sample = noisySample.pose();
        Pose2d measurement = noisyMeasurement.pose();
        IsotropicNoiseSE2 stateSigma = noisySample.noise();
        IsotropicNoiseSE2 visionSigma = noisyMeasurement.noise();

        // translation
        // the result is on the line segment between sample and measurement, so we just
        // look at the length of that segment.
        //
        // this is the start of that segment.
        VariableR1 sampleV = new VariableR1(
                0,
                stateSigma.cartesianVariance());
        // this is the end.
        VariableR1 measurementV = new VariableR1(
                measurement.getTranslation().getDistance(sample.getTranslation()),
                visionSigma.cartesianVariance());

        // for now this uses the same weighting as before.
        // TODO: use covariance inflation
        // VariableR1 cartesian = InverseVarianceWeighting.fuse(sampleV, measurementV);
        VariableR1 cartesian = CovarianceInflation.fuse(sampleV, measurementV);

        Translation2d deltaTranslation = measurement.getTranslation().minus(sample.getTranslation());
        Rotation2d deltaTranslationDirection = deltaTranslation.getAngle();

        Translation2d scaledTranslation = new Translation2d(cartesian.mean(),
                deltaTranslationDirection);
        Translation2d newTranslation = sample.getTranslation().plus(scaledTranslation);

        // rotation

        Rotation2d deltaRotation = measurement.getRotation().minus(sample.getRotation());

        VariableR1 sampleRV = new VariableR1(
                0,
                stateSigma.rotationVariance());
        VariableR1 measurementRV = new VariableR1(
                deltaRotation.getRadians(), visionSigma.rotationVariance());
        // TODO: use covariance inflation

        // VariableR1 rotation = InverseVarianceWeighting.fuse(sampleRV, measurementRV);
        VariableR1 rotation = CovarianceInflation.fuse(sampleRV, measurementRV);

        Rotation2d scaledRotation = new Rotation2d(rotation.mean());

        Rotation2d newRotation = sample.getRotation().plus(scaledRotation);

        Pose2d result = new Pose2d(newTranslation, newRotation);

        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromVariance(
                cartesian.variance(),
                rotation.variance());

        return new NoisyPose2d(result, noise);
    }

}
