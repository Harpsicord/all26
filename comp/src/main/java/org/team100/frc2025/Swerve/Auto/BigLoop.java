package org.team100.frc2025.Swerve.Auto;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactorySE2;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** big looping trajectory for testing */
public class BigLoop implements Function<Pose2d, Trajectory100> {
    private final TrajectoryPlanner m_planner;

    public BigLoop(LoggerFactory log, SwerveKinodynamics kinodynamics) {
        List<TimingConstraint> constraints = new TimingConstraintFactory(kinodynamics).auto(log.type(this));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactorySE2 pathFactory = new PathFactorySE2();
        m_planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
    }

    @Override
    public Trajectory100 apply(Pose2d p0) {
        // field-relative
        Pose2d p1 = new Pose2d(
                p0.getX() + 2,
                p0.getY() - 2,
                p0.getRotation().plus(Rotation2d.kCCW_Pi_2));
        Pose2d p2 = new Pose2d(
                p0.getX() + 4,
                p0.getY(),
                p0.getRotation().plus(Rotation2d.k180deg));
        Pose2d p3 = new Pose2d(
                p0.getX() + 2,
                p0.getY() + 2,
                p0.getRotation().plus(Rotation2d.kCW_Pi_2));
        // note adjustment of scale and rotation "direction" (i.e. rate relative to
        // translation)
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        p0,
                        new DirectionSE2(1, 0, 0),
                        1),
                new WaypointSE2(
                        p1,
                        new DirectionSE2(1, 0, 0.5),
                        1.3),
                new WaypointSE2(
                        p2,
                        new DirectionSE2(0, 1, 0.5),
                        1.3),
                new WaypointSE2(
                        p3,
                        new DirectionSE2(-1, 0, 0.5),
                        1.3),
                new WaypointSE2(
                        p0,
                        new DirectionSE2(-1, 0, 0),
                        1)
        //
        );
        return m_planner.restToRest(waypoints);
    }
}
