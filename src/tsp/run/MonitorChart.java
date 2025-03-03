package tsp.run;

import java.awt.BorderLayout;
import java.awt.Dimension;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.time.Millisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;

/**
 * @author Alexandre Blansché
 * Affichage de la fonction d'évaluation dans le temps
 */
public class MonitorChart
{
	private static MonitorChart instance = null;
	private TimeSeries bestEvaluation;
	private TimeSeries currentEvaluation;
	
	/**
	 * @return L'instance courante
	 */
	public static MonitorChart getInstance ()
	{
		MonitorChart instance = MonitorChart.instance;
		if (instance == null)
			instance = new MonitorChart ("");
		return instance;
	}
	
	/**
	 * @param title Le titre du graphique
	 * @param visible Indique s'il faut afficher le graphique
	 * @return Une nouvelle instance
	 */
	public static MonitorChart getNewInstance (String title)
	{
		MonitorChart.instance = new MonitorChart (title);
		return MonitorChart.instance;
	}
	
	private MonitorChart (String title)
	{
		if (Main.DISPLAY_CHART)
		{
			this.bestEvaluation = new TimeSeries ("Best evaluation");
			this.currentEvaluation = new TimeSeries ("Current evaluation");
			TimeSeriesCollection tsc = new TimeSeriesCollection ();
			tsc.addSeries(this.currentEvaluation);
			tsc.addSeries(this.bestEvaluation);
			JFreeChart chart = ChartFactory.createTimeSeriesChart (title, "Time", "Fitness", tsc);
			ChartPanel chartPanel = new ChartPanel (chart);
			chartPanel.setPreferredSize (new Dimension (600, 300));
			MainFrame mainFrame = MainFrame.getInstance ();
			mainFrame.add (chartPanel, BorderLayout.NORTH);
			mainFrame.pack ();
		}
	}
	
	/**
	 * Ajoute une évaluation à l'affichage graphique
	 * @param current Évaluation de la solution courante
	 * @param best Évaluation de la meilleure solution
	 */
	public void addData (double current, double best)
	{
		if (Main.DISPLAY_CHART)
		{
			try
			{
				Millisecond now = new Millisecond ();
				this.currentEvaluation.add (now, current);
				this.bestEvaluation.add (now, best);
			}
			catch (Exception e)
			{			
			}
		}
	}
}
