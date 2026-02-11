package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.uncertainty.VariableR1;

public class CovarianceInflationTest {
    private static final boolean DEBUG = true;
    private static final double DELTA = 0.001;

    @Test
    void testCrisp() {
        VariableR1 a = new VariableR1(0, 0);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        // Crisp variable wins
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0, c.variance(), DELTA);
    }

    @Test
    void testSelf() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Overconfident (higher than the minimum)
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testUnequalVariance() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 0.1);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0.091, c.variance(), DELTA);
    }

    @Test
    void testHighVariance() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        // Ignores the higher variance
        assertEquals(0.01, c.mean(), DELTA);
        // Ignores the higher variance
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testEqualVariance() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        // Equal variance -> expectation in the middle
        assertEquals(0.5, c.mean(), DELTA);
        // Respects mean dispersion (a little)
        assertEquals(0.505, c.variance(), DELTA);
    }

    @Test
    void testMinimumVariance() {
        VariableR1 a = new VariableR1(0, 0.0000001);
        VariableR1 b = new VariableR1(0, 0.00000001);
        VariableR1 c = CovarianceInflation.fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Minimum applies.
        assertEquals(9e-6, c.variance(), 1e-6);
    }

    /**
     * Update the estimate many times.
     * 
     * Output is plotted here:
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=1604242948#gid=1604242948
     */
    @Test
    void testEvolution() {
        Random random = new Random();
        // note non-zero initial mean
        // note not-huge initial variance (1 cm stddev)
        VariableR1 state = new VariableR1(0.3, 0.0001);
        double cameraStdDev = 0.03; // typical camera stddev
        double cameraVariance = cameraStdDev * cameraStdDev;
        if (DEBUG)
            System.out.println("t, camera, state, state stddev");
        for (double t = 0; t < 10; t += 0.02) {
            double cameraEstimate;
            if (t < 3)
                cameraEstimate = 0.0;
            else if (t < 6)
                cameraEstimate = 0.3;
            else
                cameraEstimate = 0.0;
            VariableR1 camera = new VariableR1(random.nextGaussian() * cameraStdDev + cameraEstimate, cameraVariance);
            // state prior to update
            if (DEBUG)
                System.out.printf("%f, %f, %f, %f\n",
                        t, camera.mean(), state.mean(), Math.sqrt(state.variance()));
            // do the update
            state = CovarianceInflation.fuse(state, camera);
        }
    }

}
