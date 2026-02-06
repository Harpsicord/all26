package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VariableR1Test {
    private static final double DELTA = 0.001;

    @Test
    void testAdd0() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.add(a, b);
        assertEquals(1, c.mean(), DELTA);
        assertEquals(2, c.variance(), DELTA);
    }

    @Test
    void testFuse0() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion has no effect: this isn't like "more evidence".
        assertEquals(1, c.variance(), DELTA);
    }

        @Test
    void testFuse0a() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse2(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion increases confidence.
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse1() {
        // Different means
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse(a, b);
        // Equal variance: mean is in the middle.
        assertEquals(0.5, c.mean(), DELTA);
        // Dispersion of the mean adds 0.25
        assertEquals(1.25, c.variance(), DELTA);
    }

    @Test
    void testWeight0() {
        // Same variance.
        double varA = 1;
        double varB = 1;
        double w = VariableR1.weight(varA, varB);
        // Equal weights.
        assertEquals(0.5, w, DELTA);
    }

}
