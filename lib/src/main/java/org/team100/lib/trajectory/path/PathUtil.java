package org.team100.lib.trajectory.path;

import org.team100.lib.trajectory.spline.SplineSE2;
import org.team100.lib.trajectory.spline.SplineSE3;
import org.team100.lib.util.Math100;

public class PathUtil {

    /**
     * Interpolates by resampling the underlying spline.
     * 
     * The parameter is assumed to vary linearly between points, so the interpolant
     * here is just a fraction of that linear variation.
     */
    public static PathSE2Entry interpolate(PathSE2Entry a, PathSE2Entry b, double x) {
        if (PathSE2Point.DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.parameter().s(), b.parameter().s());
        SplineSE2 spline = null;
        double s = 0;
        if (a.parameter().spline() == b.parameter().spline()) {
            // ok to interpolate using this spline
            if (PathSE2Point.DEBUG)
                System.out.println("same spline");
            spline = a.parameter().spline();
            s = Math100.interpolate(a.parameter().s(), b.parameter().s(), x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.parameter().s() < 1e-6) {
                // other one is the start, so use this one
                if (PathSE2Point.DEBUG)
                    System.out.println("use this spline");
                spline = a.parameter().spline();
                s = Math100.interpolate(a.parameter().s(), 1, x);
            } else {
                if (PathSE2Point.DEBUG)
                    System.out.println("use the other spline");
                spline = b.parameter().spline();
                s = Math100.interpolate(0, b.parameter().s(), x);
            }
        }
        if (PathSE2Point.DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.parameter().s(), b.parameter().s(), x, s);

        if (spline != null)
            return spline.entry(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    public static PathSE3Point interpolate(PathSE3Point a, PathSE3Point b, double x) {
        if (PathSE3Point.DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.m_s, b.m_s);
        SplineSE3 spline = null;
        double s = 0;
        if (a.m_spline == b.m_spline) {
            // ok to interpolate using this spline
            if (PathSE3Point.DEBUG)
                System.out.println("same spline");
            spline = a.m_spline;
            s = Math100.interpolate(a.m_s, b.m_s, x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.m_s < 1e-6) {
                // other one is the start, so use this one
                if (PathSE3Point.DEBUG)
                    System.out.println("use this spline");
                spline = a.m_spline;
                s = Math100.interpolate(a.m_s, 1, x);
            } else {
                if (PathSE3Point.DEBUG)
                    System.out.println("use the other spline");
                spline = b.m_spline;
                s = Math100.interpolate(0, b.m_s, x);
            }
        }
        if (PathSE3Point.DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.m_s, b.m_s, x, s);
        // sample the spline again instead of interpolating.
        if (spline != null)
            return spline.sample(s);
        return null;
    }

}
