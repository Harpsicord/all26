package org.team100.sim2026;

public class Outpost implements Actor, BallContainer {
    /** Where the balls go next */
    final Zone target;

    int count;

    public Outpost(Zone target, int initialCount) {
        this.target = target;
        count = initialCount;
    }

    @Override
    public int count() {
        return count;
    }

    /** Dump them all. */
    @Override
    public Runnable step() {
        return () -> {
            target.accept(count);
            count = 0;
        };
    }
}
