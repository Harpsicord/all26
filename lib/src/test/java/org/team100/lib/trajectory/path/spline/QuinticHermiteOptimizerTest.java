package org.team100.lib.trajectory.path.spline;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.TrajectoryPlotter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class QuinticHermiteOptimizerTest {

    @Test
    void testMulti0() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(new Translation2d(0, 100), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(new Translation2d(50, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(new Translation2d(100, 100), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(a, b));
        splines.add(new SplineSE2(b, c));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines));

    }

    @Test
    void testMulti1() {

        WaypointSE2 d = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(new Translation2d(0, 50), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(new Translation2d(100, 0), new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(d, e));
        splines.add(new SplineSE2(e, f));
        splines.add(new SplineSE2(f, g));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines));

    }

    @Test
    void testMulti2() {
        WaypointSE2 h = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(new Translation2d(50, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(new Translation2d(100, 50), new Rotation2d()),
                new DirectionSE2(1, 1, 0), 1);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(new Translation2d(150, 0), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(new Translation2d(150, -50), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);

        List<SplineSE2> splines2 = new ArrayList<>();
        splines2.add(new SplineSE2(h, i));
        splines2.add(new SplineSE2(i, j));
        splines2.add(new SplineSE2(j, k));
        splines2.add(new SplineSE2(k, l));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines2));

    }

    @Test
    void testMulti3() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(new Translation2d(0, 100), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(new Translation2d(50, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(new Translation2d(100, 100), new Rotation2d(Math.PI)),
                new DirectionSE2(0, 1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(a, b));
        splines.add(new SplineSE2(b, c));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines));

    }

    @Test
    void testMulti4() {

        WaypointSE2 d = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(new Translation2d(0, 50), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(new Translation2d(100, 50), new Rotation2d(Math.PI)),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(new Translation2d(100, 0), new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(d, e));
        splines.add(new SplineSE2(e, f));
        splines.add(new SplineSE2(f, g));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines));
    }

    @Test
    void testMulti5() {

        WaypointSE2 h = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(new Translation2d(50, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(new Translation2d(100, 50), new Rotation2d(Math.PI)),
                new DirectionSE2(1, 1, 0), 1);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(new Translation2d(150, 0), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(new Translation2d(150, -50), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(0, -1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(h, i));
        splines.add(new SplineSE2(i, j));
        splines.add(new SplineSE2(j, k));
        splines.add(new SplineSE2(k, l));

        TrajectoryPlotter.plot(new SplineToVectorSeries(5).convert("before", splines));

    }
}
