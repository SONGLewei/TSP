package tsp.projects.demo.hillclimbing;

import java.util.Random;

import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.InvalidProjectException;
import tsp.projects.DemoProject;

/**
 * @author Alexandre Blansché
 * Hill Climbing (aléatoire)
 */
public class HillClimbing extends DemoProject
{
	private Random random;
	private Path path;

	/**
	 * Méthode d'évaluation de la solution
	 * @param evaluation 
	 * @throws InvalidProjectException
	 */
	public HillClimbing (Evaluation evaluation) throws InvalidProjectException
	{
		super (evaluation);
		this.addAuthor ("Alexandre Blansché");
		this.setMethodName ("Hill Climbing");
	}

	@Override
	public void initialization ()
	{
		this.random = new Random ();
		this.path = new Path (this.problem.getLength ());
		this.evaluation.evaluate (this.path);
	}

	private Path transform (Path path)
	{
		int [] res = path.getCopyPath ();
		int i = this.random.nextInt (res.length);
		int j = this.random.nextInt (res.length);
		int tmp = res [i];
		res [i] = res [j];
		res [j] = tmp;
		return new Path (res);
	}

	@Override
	public void loop ()
	{
		Path path = this.transform (this.path);
		double evaluation = this.evaluation.evaluate(path);
		if (evaluation < this.evaluation.getBestEvaluation())
			this.path = path;
	}
}
