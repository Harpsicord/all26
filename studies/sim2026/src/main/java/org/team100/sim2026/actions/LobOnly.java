package org.team100.sim2026.actions;

public class LobOnly implements Action {

    private int n;

    public LobOnly(int n) {
        this.n = n;
    }

    @Override
    public String toString() {
        return "lob " + n;
    }

}
