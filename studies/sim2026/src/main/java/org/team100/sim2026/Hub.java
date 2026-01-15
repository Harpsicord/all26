package org.team100.sim2026;

import java.util.function.IntSupplier;

/** Hub contains balls briefly, and adjusts the score. */
public class Hub implements Actor, BallAcceptor, BallContainer {
    /** Where the balls go next */
    final Zone target;
    final Score score;
    final IntSupplier timer;
    final IntSupplier myAuto;
    final IntSupplier otherAuto;
    boolean active;
    int inside1 = 0;
    int inside2 = 0;
    int inside3 = 0;

    public Hub(
            Score score,
            Zone target,
            IntSupplier timer,
            IntSupplier myAuto,
            IntSupplier otherAuto) {
        this.score = score;
        this.target = target;
        this.timer = timer;
        this.myAuto = myAuto;
        this.otherAuto = otherAuto;
    }

    @Override
    public int count() {
        return inside1 + inside2 + inside3;
    }

    @Override
    public void accept(int count) {
        inside1 += count;
    }

    /** Shift balls through the queue. */
    @Override
    public Runnable step() {
        return () -> {
            target.accept(inside3);
            if (active)
                score.fuel(inside3);
            inside3 = inside2;
            inside2 = inside1;
            inside1 = 0;
        };
    }
}
