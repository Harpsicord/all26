package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class NudgingVisionUpdaterTest {
    private static final double DELTA = 0.001;

    /**
     * If the measurement is the same as the sample, nothing happens.
     */
    @Test
    void testZeroNudge() {
        // state is twice as confident as camera
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        NoisyPose2d sample = new NoisyPose2d(new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(new Pose2d(), visionNoise);
        NoisyPose2d nudged = NudgingVisionUpdater.nudge(sample, measurement);
        assertEquals(0, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.008944, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.008944, nudged.noise().rotation(), 1e-6);
    }

    @Test
    void testOneGentleNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.001, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.1, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d(1)), visionNoise);
        NoisyPose2d nudged = NudgingVisionUpdater.nudge(sample, measurement);
        assertEquals(0.000010, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        // infinite-variance vision update is completely ignored.
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.001000, nudged.noise().cartesian(), 1e-6);
        // vision has infinite variance; this is the state variance.
        assertEquals(0.1, nudged.noise().rotation(), 1e-6);

    }

    /**
     * Let's say the camera is correct, and it says the sample is 0.1 meters off.
     * How long does it take for the pose buffer to contain the right pose? Kind of
     * a long time.
     */
    @Test
    void testGentleNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NoisyPose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement);
        }
        // after 1 sec
        Transform2d error = measurement.pose().minus(nudged.pose());
        assertEquals(0.033, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement);
        }
        // after 2 sec
        error = measurement.pose().minus(nudged.pose());
        assertEquals(0.020, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    @Test
    void testOneFirmNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, 0.5);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NoisyPose2d nudged = NudgingVisionUpdater.nudge(sample, measurement);
        assertEquals(0.003846, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        assertEquals(0, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.009806, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.098058, nudged.noise().rotation(), 1e-6);

    }

    @Test
    void testOneFirm3dNudge() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, 0.5);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(1, 2, new Rotation2d(3)), visionNoise);
        NoisyPose2d nudged = NudgingVisionUpdater.nudge(sample, measurement);
        assertEquals(0.038461, nudged.pose().getX(), 1e-6);
        assertEquals(0.076923, nudged.pose().getY(), 1e-6);
        assertEquals(0.115385, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.009806, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.098058, nudged.noise().rotation(), 1e-6);

    }

    /**
     * Same as above with firmer updates. We want error under 2cm within about 1s.
     */
    @Test
    void testFirmerNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.1);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.05, Double.MAX_VALUE);
        NoisyPose2d sample = new NoisyPose2d(new Pose2d(), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0.1, 0, new Rotation2d()), visionNoise);
        NoisyPose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement);
        }
        // after 1 sec
        Transform2d error = measurement.pose().minus(nudged.pose());
        assertEquals(0.033, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement);
        }
        // after 2 sec
        error = measurement.pose().minus(nudged.pose());
        assertEquals(0.020, error.getX(), 1e-6);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    /** Nudge across the angle modulus */
    @Test
    void testAngleCrossing() {
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        NoisyPose2d sample = new NoisyPose2d(
                new Pose2d(0, 0, new Rotation2d(3)), stateNoise);
        NoisyPose2d measurement = new NoisyPose2d(
                new Pose2d(0, 0, new Rotation2d(-3)), visionNoise);
        NoisyPose2d nudged = NudgingVisionUpdater.nudge(sample, measurement);
        assertEquals(0, nudged.pose().getX(), 1e-6);
        assertEquals(0, nudged.pose().getY(), 1e-6);
        // nudged value is *more positive* even though the measurement is negative.
        assertEquals(3.056637, nudged.pose().getRotation().getRadians(), 1e-6);
        assertEquals(0.008944, nudged.noise().cartesian(), 1e-6);
        assertEquals(0.008944, nudged.noise().rotation(), 1e-6);

    }

}
