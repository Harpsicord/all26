package org.team100.lib.trajectory.spline;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SplineSE2ToVectorSeries {
    private static final boolean DEBUG = false;

    private static final double DS = 0.05;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public SplineSE2ToVectorSeries(double scale) {
        m_scale = scale;
    }

    /**
     * Show little arrows.
     * 
     * @return (x, y, dx, dy)
     */
    public List<VectorSeries> convert(List<SplineSE2> splines) {
        List<VectorSeries> result = new ArrayList<>();
        for (int i = 0; i < splines.size(); i++) {
            SplineSE2 spline = splines.get(i);
            VectorSeries series = new VectorSeries(String.format("%d", i));
            for (double s = 0; s <= 1.001; s += DS) {
                Pose2d p = spline.entry(s).point().waypoint().pose();
                if (DEBUG)
                    System.out.println(p);
                double x = p.getX();
                double y = p.getY();
                Rotation2d heading = p.getRotation();
                double dx = m_scale * heading.getCos();
                double dy = m_scale * heading.getSin();
                series.add(x, y, dx, dy);
            }
            result.add(series);
        }
        return result;
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
                double x = spline.entry(s).point().waypoint().pose().getX();
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
