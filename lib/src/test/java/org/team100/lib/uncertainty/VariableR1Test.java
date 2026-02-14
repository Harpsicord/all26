package org.team100.lib.uncertainty;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VariableR1Test {
    private static final double DELTA = 0.001;

    @Test
    void testAdd0() {
        VariableR1 a = VariableR1.fromVariance(0, 1);
        VariableR1 b = VariableR1.fromVariance(1, 1);
        VariableR1 c = VariableR1.add(a, b);
        assertEquals(1, c.mean(), DELTA);
        assertEquals(2, c.variance(), DELTA);
    }
}
