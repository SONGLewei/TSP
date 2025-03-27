package tsp.projects.competitor;

import java.util.Random;

import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.CompetitorProject;
import tsp.projects.InvalidProjectException;
import tsp.evaluation.Coordinates;

/**
 * @author Alexandre Blansché
 * Hill Climbing (aléatoire)
 */
public class deuxOp extends CompetitorProject
{
    private Random random;
    private Path bestRoute;
    private boolean finished = false;

    /**
     * Méthode d'évaluation de la solution
     * @param evaluation
     * @throws InvalidProjectException
     */
    public deuxOp(Evaluation evaluation) throws InvalidProjectException
    {
        super (evaluation);
        this.addAuthor ("Lewei SONG");
        this.addAuthor("Achille");
        this.setMethodName ("Hill Climbing");
    }

    @Override
    public void initialization ()
    {
        this.random = new Random();
        int init = this.random.nextInt(this.problem.getLength());
        int n = this.problem.getLength();
        this.bestRoute = gloutonAlgo(n,init);
        this.evaluation.evaluate(this.bestRoute);
    }

    @Override
    public void loop ()
    {
        if(this.finished){
            Thread.currentThread().interrupt();
            return;
        }

        boolean improved = true;
        int n = this.problem.getLength();
        int[] route = this.bestRoute.getPath();
        while(improved) {
            improved = false;
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    if (i == 0 && j == n - 1) continue;

                    int cityA = route[i];
                    int cityB = route[(i + 1) % n];
                    int cityC = route[j];
                    int cityD = route[(j + 1) % n];

                    Coordinates coordA = this.problem.getCoordinates(cityA);
                    Coordinates coordB = this.problem.getCoordinates(cityB);
                    Coordinates coordC = this.problem.getCoordinates(cityC);
                    Coordinates coordD = this.problem.getCoordinates(cityD);

                    double currentDist = coordA.distance(coordB) + coordC.distance(coordD);
                    double newDist = coordA.distance(coordC) + coordB.distance(coordD);

                    if (newDist < currentDist) {
                        reserve(route, i + 1, j);

                        this.bestRoute = new Path(route);

                        this.evaluation.evaluate(this.bestRoute);

                        improved = true;
                        break;
                    }
                }
                if (improved) break;
            }
        }
        this.finished = true;
        Thread.currentThread().interrupt();

    }

    private void reserve(int[] route, int from, int to) {
        while (from<to){
            int temp = route[from];
            route[from] = route[to];
            route[to] = temp;
            from++;
            to--;
        }
    }

    private Path gloutonAlgo(int n,int init) {
        int[] route = new int[n];
        boolean[] visited = new boolean[n];

        route[0] = init;
        visited[init] = true;

        for(int i = 1;i<n;i++){
            int prev = route[i-1];
            Coordinates coordPrev = this.problem.getCoordinates(prev);

            int nearest = -1;
            double bestDist = Double.POSITIVE_INFINITY;
            for(int j=0;j<n;j++){
                if(!visited[j]){
                    Coordinates coordJ = this.problem.getCoordinates(j);
                    double dist = coordPrev.distance(coordJ);
                    if(dist<bestDist){
                        bestDist = dist;
                        nearest = j;
                    }
                }
            }

            route[i] = nearest;
            visited[nearest] = true;
        }

        return new Path(route);
    }
}