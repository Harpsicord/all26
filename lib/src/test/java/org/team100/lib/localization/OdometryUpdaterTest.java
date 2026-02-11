package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Rotation2d;

public class OdometryUpdaterTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest(log);

    private SwerveModulePositions positions;

    @Test
    void testNewState1() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions);
        // previous state is at zero, but uncertain
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // measurements haven't moved
        Rotation2d gyroYaw = new Rotation2d();

        // result shouldn't move.
        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), DELTA);
        // variance shouldn't change.
        assertEquals(1, newState.noise().cartesian(), DELTA);
        assertEquals(1, newState.noise().rotation(), DELTA);
        // there's no movement in yaw, so bias is unchanged.
        assertEquals(0, newState.gyroBias().mean(), DELTA);
        assertEquals(0.001, newState.gyroBias().sigma(), DELTA);
    }

    @Test
    void testNewState2() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions);

        // previous state is at zero, pretty sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // 0.1m ahead (this is max speed)
        Rotation2d gyroYaw = new Rotation2d();
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)));

        // result shouldn't be 0.1 ahead
        // high speed means considerable added variance
        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0.1, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, newState.noise().cartesian(), DELTA);
        assertEquals(0.012, newState.noise().rotation(), DELTA);
    }

}
