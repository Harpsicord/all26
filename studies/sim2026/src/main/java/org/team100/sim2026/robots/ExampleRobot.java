package org.team100.sim2026.robots;

import java.util.function.IntSupplier;

import org.team100.sim2026.Hub;
import org.team100.sim2026.Zone;

/** Ferry during our active time, otherwise lob. */
public class ExampleRobot extends Robot {

    public ExampleRobot(
            String name,
            Zone myZone,
            Zone neutralZone,
            Zone otherZone,
            Hub myHub,
            int initialCount,
            IntSupplier matchTimer) {
        super(name, myZone, neutralZone, otherZone, myHub, initialCount, matchTimer);
    }

    void action() {
        if (matchTimer.getAsInt() < 20) {
            // auton
            ferry();
        } else {
            // teleop
            if (active) {
                ferry();
            } else {
                lob();
            }
        }
    }

}
