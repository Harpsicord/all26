package org.team100.frc2025;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Stream;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.Swerve.Auto.GoToCoralStation;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.CoralStation;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.prr.ElevatorArmWristKinematics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2Planner;
import org.team100.lib.trajectory.TrajectorySE2ToVectorSeries;
import org.team100.lib.trajectory.path.PathFactorySE2;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TorqueConstraint;
import org.team100.lib.trajectory.timing.TrajectorySE2Factory;
import org.team100.lib.trajectory.timing.YawRateConstraint;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;

/** Show some trajectories from 2025 in a vector series chart. */
public class TrajectoryGallery {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(log);

    @Test
    void testGoToCoralStation1() {
        List<VectorSeries> right = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        ChartUtil.plotOverlay(right);
    }

    @Test
    void testGoToCoralStation2() {
        List<VectorSeries> left = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        ChartUtil.plotOverlay(left);
    }

    @Test
    void testGoToCoralStation3() {
        List<VectorSeries> iLeft = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        List<VectorSeries> kLeft = series(CoralStation.LEFT, ReefPoint.K, ScoringLevel.L4);
        List<VectorSeries> lLeft = series(CoralStation.LEFT, ReefPoint.L, ScoringLevel.L4);
        List<VectorSeries> fRight = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        List<VectorSeries> dRight = series(CoralStation.RIGHT, ReefPoint.D, ScoringLevel.L4);
        List<VectorSeries> cRight = series(CoralStation.RIGHT, ReefPoint.C, ScoringLevel.L4);
        ChartUtil.plotOverlay(Stream.of(iLeft, kLeft, lLeft, fRight, dRight, cRight)
                .flatMap(Collection::stream).toList());
    }

    @Test
    void testCalgames() {
        // homeToL2
        List<TimingConstraint> c = new ArrayList<>();

        // These are known to work, but suboptimal.
        c.add(new ConstantConstraint(log, 10, 5));
        c.add(new YawRateConstraint(log, 10, 5));
        // This is new
        c.add(new TorqueConstraint(20));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathFactorySE2 pathFactory = new PathFactorySE2(0.05, 0.01, 0.01, 0.1);
        TrajectorySE2Planner m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        ElevatorArmWristKinematics m_kinematics = new ElevatorArmWristKinematics(0.5, 0.343);
        Pose2d m_home = m_kinematics.forward(CalgamesMech.HOME);

        WaypointSE2 m_goal = WaypointSE2.irrotational(CalgamesMech.L2, 1.5, 1.2);

        DirectionSE2 m_course = DirectionSE2.irrotational(1.5);
        WaypointSE2 m_currentPose = new WaypointSE2(m_home, m_course, 1);
        TrajectorySE2 m_trajectory = m_planner.restToRest(List.of(m_currentPose, m_goal));
        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.1).convert(m_trajectory);
        ChartUtil.plotOverlay(series);

    }

    private List<VectorSeries> series(CoralStation coralStation, ReefPoint reefPoint, ScoringLevel scoringLevel) {
        GoToCoralStation toStation = new GoToCoralStation(log, swerveKinodynamics, coralStation, 0.5);
        // the start is the goal from the previous maneuver
        Pose2d start = FieldConstants.makeGoal(scoringLevel, reefPoint);
        TrajectorySE2 trajectory = toStation.apply(start);
        return new TrajectorySE2ToVectorSeries(0.1).convert(trajectory);
    }

}
