package org.team100.sim2026.actions;

import org.team100.sim2026.Tower;

public class Climb implements Action {
    final Tower tower;
    int timer = 0;

    public Climb(Tower tower, int duration) {
        this.tower = tower;
        timer = duration;
    }

    public void step() {
        // System.out.println("step " + timer);
        timer--;
        if (timer == 0) {
            tower.climbLevel3();
        }
    }

    public boolean done() {
        return !(timer > 0);
    }

    @Override
    public String toString() {
        return "climb";
    }

}
