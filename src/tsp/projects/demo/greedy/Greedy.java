package tsp.projects.demo.greedy;

import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.InvalidProjectException;
import tsp.projects.DemoProject;

/**
 * @author Alexandre Blansché
 * Consstruction gloutonne
 */
public class Greedy extends DemoProject
{
	private Random random;
	private Path path;
	private int length;

	/**
	 * Méthode d'évaluation de la solution
	 * @param evaluation 
	 * @throws InvalidProjectException
	 */
	public Greedy (Evaluation evaluation) throws InvalidProjectException
	{
		super (evaluation);
		this.addAuthor ("Alexandre Blansché");
		this.setMethodName ("Greedy");
	}

	private Path gluttonDeterministicPath (int length)
	{
		int [] path = new int [length];
		boolean [] used = new boolean [length];
		int init = this.random.nextInt (length);
		path [0] = init;
		used [init] = true;
		for (int i = 1; i < length; i++)
		{
			Coordinates current = this.problem.getCoordinates (path [i - 1]);
			double minDist = Double.MAX_VALUE;
			int nn = -1;
			for (int j = 0; j < length; j++)
			{
				if (!used [j])
				{
					Coordinates tmp = this.problem.getCoordinates (j);
					double dist = current.distance (tmp);
					if (dist < minDist)
					{
						minDist = dist;
						nn = j;
					}
				}
			}
			path [i] = nn;
			used [nn] = true;
		}
		return new Path (path);
	}

	@Override
	public void initialization ()
	{
		this.random = new Random ();
		this.length = this.problem.getLength ();
	}

	@Override
	public void loop ()
	{
		this.path = this.gluttonDeterministicPath(this.length);
		this.evaluation.evaluate (this.path);
	}
}
