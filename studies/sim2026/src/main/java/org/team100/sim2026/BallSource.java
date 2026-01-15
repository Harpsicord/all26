package org.team100.sim2026;

public interface BallSource {
    /** Reduce the count by up to count, return the number actually taken. */
    int take(int count);
}
