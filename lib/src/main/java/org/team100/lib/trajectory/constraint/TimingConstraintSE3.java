package org.team100.lib.trajectory.constraint;

import org.team100.lib.trajectory.path.PathSE3Point;

public interface TimingConstraintSE3 {
    double maxV(PathSE3Point state);

    double maxAccel(PathSE3Point state, double velocityM_S);

    double maxDecel(PathSE3Point state, double velocityM_S);

}
