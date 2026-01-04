package org.team100.lib.trajectory;

import java.awt.Dimension;
import java.awt.Frame;
import java.util.List;

import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.VectorSeriesCollection;
import org.jfree.data.xy.XYDataset;
import org.team100.lib.util.ChartUtil;

public class TrajectoryPlotter {
    public static final boolean SHOW = false;
    private static final int SIZE = 500;

    /** plot each xy dataset as its own chart in a vertical list. */
    public static void plotStacked(XYDataset... dataSets) {
        if (!SHOW)
            return;

        XYItemRenderer renderer = new StandardXYItemRenderer(StandardXYItemRenderer.SHAPES);

        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));
        for (XYDataset dataSet : dataSets) {
            XYPlot plot = new XYPlot(
                    dataSet, new NumberAxis("X"), new NumberAxis("Y"), renderer);

            NumberAxis domain = (NumberAxis) plot.getDomainAxis();
            NumberAxis range = (NumberAxis) plot.getRangeAxis();
            domain.setRangeWithMargins(ChartUtil.xRange(dataSet));
            range.setRangeWithMargins(ChartUtil.yRange(dataSet));

            ChartPanel panel = new ChartPanel(new JFreeChart(plot));
            panel.setPreferredSize(new Dimension(SIZE, SIZE / dataSets.length));
            frame.add(panel);
        }

        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    }

    /** Plot multiple vector series on the same axes. */
    public static void plotOverlay(List<VectorSeries> seriesList) {
        if (!SHOW)
            return;

        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));

        XYPlot plot = new XYPlot();

        plot.setDomainAxis(new NumberAxis("X"));
        plot.setRangeAxis(new NumberAxis("Y"));

        Range xRange = null;
        Range yRange = null;
        for (int i = 0; i < seriesList.size(); ++i) {
            VectorSeries series = seriesList.get(i);
            VectorSeriesCollection dataSet = ChartUtil.getDataSet(series);
            plot.setDataset(i, dataSet);
            XYItemRenderer renderer = new VectorRenderer();

            plot.setRenderer(i, renderer);
            if (xRange == null) {
                xRange = ChartUtil.xRange(dataSet);
                yRange = ChartUtil.yRange(dataSet);
            } else {
                xRange = Range.combine(xRange, ChartUtil.xRange(dataSet));
                yRange = Range.combine(yRange, ChartUtil.yRange(dataSet));
            }
        }

        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        domain.setRangeWithMargins(xRange);
        range.setRangeWithMargins(yRange);

        ChartPanel panel = new ChartPanel(new JFreeChart(plot));
        panel.setPreferredSize(new Dimension(SIZE, SIZE));
        frame.add(panel);

        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

    }

}
