package org.team100.sim2026;

// keeps score
public class Tower {
    final Score score;

    public Tower(Score score) {
        this.score = score;
    }

    public void climbLevel3() {
        score.climbLevel3();
    }

}
