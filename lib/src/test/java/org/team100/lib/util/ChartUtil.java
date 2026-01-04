package org.team100.lib.util;

import org.jfree.data.Range;
import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.VectorSeriesCollection;
import org.jfree.data.xy.XYDataset;

public class ChartUtil {

    public static Range yRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double y = dataset.getYValue(0, i);
            if (y + 0.1 > max)
                max = y + 0.1;
            if (y - 0.1 < min)
                min = y - 0.1;
        }
        return new Range(min, max);
    }

    public static Range xRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double x = dataset.getXValue(0, i);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
        }
        return new Range(min, max);
    }

    public static VectorSeriesCollection getDataSet(VectorSeries series) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        dataSet.addSeries(series);
        return dataSet;
    }
    
}
