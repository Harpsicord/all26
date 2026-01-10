package org.team100.lib.trajectory.spline;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;

public interface ISplineSE2 {
    Pose2d pose(double s);

    Vector<N2> K(double s);
}
