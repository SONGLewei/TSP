package tsp.run;

import java.awt.BorderLayout;
import java.awt.Dimension;

import javax.swing.SwingUtilities;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import tsp.evaluation.Path;
import tsp.evaluation.Problem;

/**
 * @author Alexandre Blansché
 * Affichage de la fonction d'évaluation dans le temps
 */
public class PathChart
{
	private static PathChart instance = null;
	private Problem problem;
	private XYSeries coords;

	/**
	 * @return L'instance courante
	 */
	public static PathChart getInstance ()
	{
		PathChart instance = PathChart.instance;
		if (instance == null)
			instance = new PathChart (null);
		return instance;
	}

	/**
	 * @param title Le titre du graphique
	 * @param visible Indique s'il faut afficher le graphique
	 * @return Une nouvelle instance
	 */
	public static PathChart getNewInstance (Problem problem)
	{
		PathChart.instance = new PathChart (problem);
		return PathChart.instance;
	}

	private PathChart (Problem problem)
	{
		if (Main.DISPLAY_CHART)
		{
			this.problem= problem;
			this.coords = new XYSeries ("Path", false);
			if (problem != null)
			{
				double [][] data = this.problem.getData();
				for (int i = 0; i < data.length; i++)
					this.coords.add(data [i][0], data [i][1]);
				this.coords.add(data [0][0], data [0][1]);
			}
			XYSeriesCollection xysc = new XYSeriesCollection();
			xysc.addSeries(this.coords);
			JFreeChart chart = ChartFactory.createScatterPlot(this.problem.getName(), "", "", xysc);
			XYPlot plot = (XYPlot) chart.getPlot();
			XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
			renderer.setSeriesLinesVisible(0, true);
			plot.setRenderer(renderer);
			ChartPanel chartPanel = new ChartPanel (chart);
			chartPanel.setPreferredSize (new Dimension (400, 400));
			MainFrame mainFrame = MainFrame.getInstance ();
			mainFrame.add (chartPanel, BorderLayout.SOUTH);
			mainFrame.pack ();
		}
	}

	/**
	 * Ajoute une évaluation à l'affichage graphique
	 * @param current Évaluation de la solution courante
	 * @param best Évaluation de la meilleure solution
	 */
	public void changePath (Path path)
	{
		if (Main.DISPLAY_CHART)
		{
			try
			{
				Runnable updateData = new Runnable ()
				{
					public void run ()
					{
						try
						{
							PathChart chart = PathChart.getInstance ();
							chart.coords.clear ();
							int [] p = path.getPath ();
							double [][] data = chart.problem.getData ();
							for (int i = 0; i < data.length; i++)
								chart.coords.add (data [p [i]][0], data [p [i]][1]);
							chart.coords.add (data [p [0]][0], data [p [0]][1]);
						}
						catch (Exception e)
						{
						}
					}
				};
				SwingUtilities.invokeLater (updateData);
			}
			catch (Exception e)
			{
			}
		}
	}
}
