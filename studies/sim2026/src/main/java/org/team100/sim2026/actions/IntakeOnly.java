package org.team100.sim2026.actions;

public class IntakeOnly implements Action {

    private int taken;

    public IntakeOnly(int taken) {
        this.taken = taken;
    }

    @Override
    public String toString() {
        return String.format("intake %d", taken);
    }

}
