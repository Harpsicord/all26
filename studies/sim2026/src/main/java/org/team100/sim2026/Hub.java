package org.team100.sim2026;

/** Hub contains balls briefly, and adjusts the score. */
public class Hub implements Actor, BallAcceptor, BallContainer {
    /** Where the balls go next */
    final Zone target;
    final Score score;
    boolean active;
    int inside1 = 0;
    int inside2 = 0;
    int inside3 = 0;

    public Hub(Score score, Zone target) {
        this.score = score;
        this.target = target;
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
