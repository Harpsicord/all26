package org.team100.sim2026;

/** A zone is a floor area that can contain balls. */
public class Zone implements BallAcceptor, BallContainer, BallSource {
    public final String name;
    int count;

    public Zone(String name, int initialCount) {
        this.name = name;
        count = initialCount;
    }

    @Override
    public int count() {
        return count;
    }

    @Override
    public void accept(int count) {
        this.count += count;
    }

    @Override
    public int take(int count) {
        if (count > this.count) {
            int actual = this.count;
            this.count = 0;
            return actual;
        }
        this.count -= count;
        return count;
    }

}
