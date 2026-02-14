package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.uncertainty.VariableR1;

public class InverseVarianceWeightingTest {
    private static final double DELTA = 0.001;

    @Test
    void testCrisp() {
        VariableR1 a = VariableR1.fromVariance(0, 0);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new InverseVarianceWeighting().fuse(a, b);
        // Crisp variable wins
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0, c.variance(), DELTA);
    }

    @Test
    void testSelf() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(0, 1);
        VariableR1 c = new InverseVarianceWeighting().fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion increases confidence (too much)
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testHighVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 100);
        VariableR1 c = new InverseVarianceWeighting().fuse(a, b);
        // Mostly ignores the uncertain input
        assertEquals(0.01, c.mean(), DELTA);
        // Note the variance here is about the same
        // but *less* than the previous, which is wrong.
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testEqualVariance() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = new InverseVarianceWeighting().fuse(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // Variance ignores mean dispersion
        assertEquals(0.5, c.variance(), DELTA);
    }
}
