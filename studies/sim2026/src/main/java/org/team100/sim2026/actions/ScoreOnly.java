package org.team100.sim2026.actions;

public class ScoreOnly implements Action {

    private int n;

    public ScoreOnly(int n) {
        this.n = n;
    }

    @Override
    public String toString() {
        return "score " + n;
    }
}
