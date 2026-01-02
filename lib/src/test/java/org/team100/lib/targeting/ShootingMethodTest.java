package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR2;

import edu.wpi.first.math.geometry.Translation2d;

public class ShootingMethodTest {
    private static final boolean DEBUG = true;

    @Test
    void testJacobian() {

    }

    /** For parabolic paths we can check against the analytical solution */
    @Test
    void testMotionlessParabolic() {
        double g = 9.81;
        Drag d = new Drag(0, 0, 0, 1, 0);
        // this velocity is too high for either direct or indirect fire,
        // given the limits on elevation
        double v = 5;
        Range range = new Range(d, v, 0);
        // tight tolerance for testing
        // note this tolerance is smaller than the range accuracy
        ShootingMethod m = new ShootingMethod(range, 0.001);
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.001);
        // indirect fire
        assertEquals(1.120, x.elevation().getRadians(), 0.001);
        double elevation = 1.120;
        // check the range solution
        FiringSolution s = range.get(x.elevation().getRadians());
        assertEquals(2, s.range(), 0.001);
        assertEquals(0.917, s.tof(), 0.001);
        // verify the parabola
        double R = v * v * Math.sin(2 * elevation) / g;
        double t = 2 * v * Math.sin(elevation) / g;
        assertEquals(1.999, R, 0.001);
        assertEquals(0.917, t, 0.001);
    }

    @Test
    void testMotionless() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        // note this tolerance is smaller than the range accuracy
        ShootingMethod m = new ShootingMethod(range, 0.001);
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.001);
        // indirect fire
        assertEquals(1.243, x.elevation().getRadians(), 0.001);
        // check the range solution
        FiringSolution s = range.get(x.elevation().getRadians());
        assertEquals(2, s.range(), 0.001);
        assertEquals(1.208, s.tof(), 0.001);
    }

    // TODO: why is this broken
    @Test
    void testTowardsTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving towards the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(1, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.000001);
        // lower elevation
        assertEquals(0.146, x.elevation().getRadians(), 0.001);
        // check the range solution
        FiringSolution s = range.get(x.elevation().getRadians());
        // closer range
        assertEquals(1.846, s.range(), 0.001);
        // less time
        assertEquals(0.153, s.tof(), 0.001);
    }

    @Test
    void testAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving away from the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(-2, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.000001);
        // higher elevation
        assertEquals(0.645, x.elevation().getRadians(), 0.001);
        // check the range solution
        FiringSolution s = range.get(x.elevation().getRadians());
        // longer range
        assertEquals(3.561, s.range(), 0.001);
        // more time
        assertEquals(0.781, s.tof(), 0.001);
    }

    @Test
    void testImpossible() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving fast away from the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(-10, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        assertTrue(o.isEmpty());
    }

    /**
     * TODO: this only works for indirect fire. why?
     */
    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.01);
        Translation2d robotPosition = new Translation2d();
        // driving to the left
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(0, 2);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        // lag the target (to the right)
        assertEquals(-0.826, x.azimuth().getRadians(), 0.001);
        // a bit higher elevation
        assertEquals(1.016, x.elevation().getRadians(), 0.001);
        // check the range solution
        FiringSolution s = range.get(x.elevation().getRadians());
        // a bit longer range
        assertEquals(2.95, s.range(), 0.001);
        // a bit more time
        assertEquals(1.085, s.tof(), 0.001);
    }

    /**
     * With Range caching off, on my machine this solves in 157 us,
     * so the RoboRIO could probably do it in 600 us.
     * 
     * With caching on, it takes 10 us on my machine, so maybe 50 us
     * on the roboRIO, so 10x savings, but not a big number either way.
     */
    @Test
    void testPerformance() {

        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);

        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        int iterations = 10000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            // this solve takes 4 iterations
            m.solve(robotPosition, robotVelocity, targetPosition, targetVelocity);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }

    }

}
