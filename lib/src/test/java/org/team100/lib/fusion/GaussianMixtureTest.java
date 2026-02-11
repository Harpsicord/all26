package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.uncertainty.VariableR1;

public class GaussianMixtureTest {
    private static final double DELTA = 0.001;

    @Test
    void testCrisp() {
        VariableR1 a = VariableR1.fromVariance(0, 0);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new GaussianMixture().fuse(a, b);
        // Crisp variable wins
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0, c.variance(), DELTA);
    }

    @Test
    void testSelf() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(0, 1);
        VariableR1 c = new GaussianMixture().fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion has no effect: this isn't like "more evidence".
        assertEquals(1, c.variance(), DELTA);
    }

    @Test
    void testHighVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 100);
        VariableR1 c = new GaussianMixture().fuse(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        // This is trying to *include* (some of) the high variance
        // instead of ignoring it.
        assertEquals(1.99, c.variance(), DELTA);
    }

    @Test
    void testEqualVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new GaussianMixture().fuse(a, b);
        // Equal variance: mean is in the middle.
        assertEquals(0.5, c.mean(), DELTA);
        // Dispersion of the mean adds 0.25
        assertEquals(1.25, c.variance(), DELTA);
    }

    @Test
    void testUnequalVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 0.1);
        VariableR1 c = new GaussianMixture().fuse(a, b);
        assertEquals(0.909, c.mean(), DELTA);
        assertEquals(0.264, c.variance(), DELTA);
    }

}
