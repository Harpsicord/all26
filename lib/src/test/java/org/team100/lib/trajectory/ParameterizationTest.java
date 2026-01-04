package org.team100.lib.trajectory;

import java.util.List;

import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.PathPointSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.trajectory.path.PathFactorySE2;
import org.team100.lib.trajectory.path.spline.SplineSE2;
import org.team100.lib.trajectory.path.spline.SplineToVectorSeries;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** To visualize the different ways to parameterize a spline. */
public class ParameterizationTest {
    private static final boolean DEBUG = false;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * Shows x as a function of the spline parameter, s.
     */
    @Test
    void testSplineStraight() {
        // a straight line in x, since the direction is also +x
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 0.001),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 0.001));
        SplineToVectorSeries splineConverter = new SplineToVectorSeries(0.1);
        TrajectoryPlotter.plotOverlay(splineConverter.convert(List.of(spline)));

        XYSeries sx = SplineToVectorSeries.x("x", List.of(spline));
        XYSeries sxPrime = SplineToVectorSeries.xPrime("xprime", List.of(spline));
        XYSeries sxPrimePrime = SplineToVectorSeries.xPrimePrime("xprimeprime", List.of(spline));

        XYDataset d1 = new XYSeriesCollection(sx);
        XYDataset d2 = new XYSeriesCollection(sxPrime);
        XYDataset d3 = new XYSeriesCollection(sxPrimePrime);

        TrajectoryPlotter.plotStacked(d1, d2, d3);
    }

    /**
     * Shows x as a function of the spline parameter, s.
     */
    @Test
    void testSplineCurved() {
        // a straight line in x, since the direction is also +x
        // note the zero scale here to force zero velocity at the ends
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1));

        SplineToVectorSeries splineConverter = new SplineToVectorSeries(0.1);
        TrajectoryPlotter.plotOverlay(splineConverter.convert(List.of(spline)));

        XYSeries sx = SplineToVectorSeries.x("x", List.of(spline));
        XYSeries sxPrime = SplineToVectorSeries.xPrime("xprime", List.of(spline));
        XYSeries sxPrimePrime = SplineToVectorSeries.xPrimePrime("xprimeprime",
                List.of(spline));

        XYDataset d1 = new XYSeriesCollection(sx);
        XYDataset d2 = new XYSeriesCollection(sxPrime);
        XYDataset d3 = new XYSeriesCollection(sxPrimePrime);

        TrajectoryPlotter.plotStacked(d1, d2, d3);
    }

    /**
     * Show x as a function of the pose list index.
     * The pose list has no parameter, it's just a list
     */
    @Test
    void testPoses() {
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 1));

        PathFactorySE2 pathFactory = new PathFactorySE2(0.1, 0.02, 0.2, 0.1);
        List<PathPointSE2> poses = pathFactory.samplesFromSplines(List.of(spline));

        XYSeries sx = PathToVectorSeries.x("spline", poses);
        XYDataset dataSet = new XYSeriesCollection(sx);
        TrajectoryPlotter.plotStacked(dataSet);
    }

    @Test
    void testTrajectory() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 2, 0.5),
                new YawRateConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactorySE2 pathFactory = new PathFactorySE2();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1));
        TrajectorySE2 trajectory = p.generateTrajectory(waypoints, 0, 0);

        // this is wrong somehow
        if (DEBUG)
            System.out.printf("TRAJECTORY\n%s\n", trajectory);

        TrajectoryToVectorSeries converter = new TrajectoryToVectorSeries(0.1);
        TrajectoryPlotter.plotOverlay(converter.convert(trajectory));

        // XYSeries tx = TrajectoryToVectorSeries.x("x", trajectory);
        // XYSeries txDot = TrajectoryToVectorSeries.xdot("xdot", trajectory);
        // XYDataset d1 = TrajectoryPlotter.collect(tx);
        // XYDataset d2 = TrajectoryPlotter.collect(txDot);

        // TrajectoryPlotter.actuallyPlot("trajectory", renderer, d1, d2);

    }

}
