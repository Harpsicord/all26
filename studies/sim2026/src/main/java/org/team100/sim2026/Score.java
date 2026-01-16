package org.team100.sim2026;

import java.util.function.Supplier;

public class Score {
    final Supplier<GamePhase> phase;
    int autoFuel;
    int teleopFuel;

    public Score(Supplier<GamePhase> phase) {
        this.phase = phase;
    }

    public void fuel(int count) {
        if (phase.get() == GamePhase.AUTO) {
            autoFuel += count;
        } else {
            teleopFuel += count;
        }
    }

    public int auto() {
        return autoFuel;
    }

    public int total() {
        return autoFuel + teleopFuel;
    }

    @Override
    public String toString() {
        return String.format("FUEL: auto %d teleop %d", autoFuel, teleopFuel);
    }

}
