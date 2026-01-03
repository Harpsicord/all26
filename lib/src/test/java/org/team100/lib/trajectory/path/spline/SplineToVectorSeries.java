package org.team100.lib.trajectory.path.spline;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SplineToVectorSeries {
    private static final boolean DEBUG = false;

    private static final double DS = 0.05;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public SplineToVectorSeries(double scale) {
        m_scale = scale;
    }

    /**
     * Show little arrows.
     * 
     * @return (x, y, dx, dy)
     */
    public VectorSeries convert(String name, List<SplineSE2> splines) {
        VectorSeries series = new VectorSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += DS) {
                Pose2d p = spline.sample(s).waypoint().pose();
                if (DEBUG)
                    System.out.println(p);
                double x = p.getX();
                double y = p.getY();
                Rotation2d heading = p.getRotation();
                double dx = m_scale * heading.getCos();
                double dy = m_scale * heading.getSin();
                series.add(x, y, dx, dy);
            }

        }
        return series;
    }

    /**
     * X as a function of s.
     * 
     * @return (s, x)
     */
    public static XYSeries x(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += DS) {
                double x = spline.sample(s).waypoint().pose().getX();
                series.add(s, x);
            }
        }
        return series;
    }

    /**
     * X prime: dx/ds, as a function of s.
     * 
     * @return (s, x')
     */
    public static XYSeries xPrime(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += DS) {
                double x = spline.dx(s);
                series.add(s, x);
            }
        }
        return series;
    }

    /**
     * X prime prime: d^2x/ds^2, as a function of s.
     * 
     * @return (s, x'')
     */
    public static XYSeries xPrimePrime(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += DS) {
                double x = spline.ddx(s);
                series.add(s, x);
            }
        }
        return series;
    }

}
