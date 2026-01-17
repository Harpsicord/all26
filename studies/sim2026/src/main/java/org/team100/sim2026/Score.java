package org.team100.sim2026;

import java.util.function.Supplier;

public class Score {
    final Supplier<GamePhase> phase;
    int autoFuel;
    int teleopFuel;
    int autoTowerLevel1;
    int teleopTowerLevel1;
    int teleopTowerLevel2;
    int teleopTowerLevel3;

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

    public void climbLevel3() {
        if (phase.get() == GamePhase.AUTO) {
            autoTowerLevel1 += 15;
        } else {
            // System.out.println("climb score");
            teleopTowerLevel3 += 30;
        }
    }

    /** auto fuel score */
    public int auto() {
        return autoFuel;
    }

    public int totalFuel() {
        return autoFuel + teleopFuel;
    }

    public int totalClimb() {
        return autoTowerLevel1
                + teleopTowerLevel1
                + teleopTowerLevel2
                + teleopTowerLevel3;
    }

    /** total score */
    public int total() {
        return totalFuel() + totalClimb();
    }

    @Override
    public String toString() {
        StringBuilder b = new StringBuilder();
        b.append(String.format("TOTAL %4d", total()));
        b.append(" ");
        b.append(String.format("FUEL: auto %4d teleop %4d",
                autoFuel, teleopFuel));
        b.append(" ");
        b.append(String.format("CLIMB: auto L1 %2d teleop L1 %2d L2 %2d L3 %2d",
                autoTowerLevel1, teleopTowerLevel1, teleopTowerLevel2, teleopTowerLevel3));
        return b.toString();
    }

}
