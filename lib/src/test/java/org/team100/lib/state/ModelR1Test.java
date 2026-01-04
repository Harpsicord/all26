package org.team100.lib.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ModelR1Test {
    private static final double DELTA = 0.001;

    @Test
    void testInterpolation1() {
        ModelR1 s0 = new ModelR1();
        ModelR1 s1 = new ModelR1(1, 0);
        ModelR1 lerp = s0.interpolate(s1, 0.5);
        assertEquals(0.5, lerp.x(), DELTA);
        assertEquals(0, lerp.v(), DELTA);
    }

    @Test
    void testInterpolation2() {
        // note this is *just* linear interpolation, it doesn't try to do the right
        // thing in phase space.
        ModelR1 s0 = new ModelR1();
        ModelR1 s1 = new ModelR1(1, 1);
        ModelR1 lerp = s0.interpolate(s1, 0.5);
        assertEquals(0.5, lerp.x(), DELTA);
        assertEquals(0.5, lerp.v(), DELTA);
    }
}
