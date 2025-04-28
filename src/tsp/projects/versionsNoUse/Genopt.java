package tsp.projects.versionsNoUse;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * @author Lewei SONG & Achille
 * Version combinant Algorithme Génétique + 2-opt :
 * - Dans initialization(), on exécute un GA pour obtenir une solution initiale
 * - Puis dans loop(), on applique 2-opt pour l'améliorer
 */
public class Genopt extends DemoProject
{
    private Random random;
    private Path bestRoute;
    private boolean finished = false;

    // ----- Paramètres du GA -----
    private static final int GA_POP_SIZE = 40;       // Taille de la population
    private static final int GA_MAX_GEN = 13;       // Nombre de générations
    private static final double GA_CROSSOVER_RATE = 0.8;
    private static final double GA_MUTATION_RATE  = 0.02;
    private static final int GA_TOURNAMENT_SIZE   = 4;

    public Genopt(Evaluation evaluation) throws InvalidProjectException
    {
        super (evaluation);
        this.addAuthor ("Lewei SONG");
        this.addAuthor("Achille");
        this.setMethodName ("Genetic+2opt");
    }

    @Override
    public void initialization ()
    {
        this.random = new Random();

        // 1) On utilise d'abord un algorithme génétique pour trouver un chemin initial
        this.bestRoute = runGeneticAlgorithm();

        // 2) On évalue ce chemin et le stocke comme "bestRoute"
        this.evaluation.evaluate(this.bestRoute);
    }

    @Override
    public void loop ()
    {


        boolean improved = true;
        int n = this.problem.getLength();
        int[] route = this.bestRoute.getPath();

        // Implémentation 2-opt identique à ton code existant
        while(improved) {
            improved = false;
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    // Éviter le cas i=0 et j=n-1 pour ne pas briser la boucle
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
                        reverseSegment(route, i + 1, j);

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
    }

    /**
     * runGeneticAlgorithm:
     * Lance un petit GA sur GA_POP_SIZE individus, pendant GA_MAX_GEN générations.
     * Retourne le meilleur Path trouvé.
     */
    private Path runGeneticAlgorithm()
    {
        // 1) Initialiser la population aléatoire
        ArrayList<Path> population = new ArrayList<>();
        for(int i=0; i<GA_POP_SIZE; i++){
            Path p = new Path(this.problem.getLength());
            this.evaluation.evaluate(p); // on évalue pour mise à jour best
            population.add(p);
        }

        // 2) Boucle de GA
        Path bestIndiv = getBest(population);
        for(int gen=0; gen<GA_MAX_GEN; gen++){

            ArrayList<Path> newPop = new ArrayList<>();
            // Elitisme: on garde le meilleur
            Path elite = getBest(population);
            newPop.add(copyOf(elite));

            // Compléter la population
            while(newPop.size() < GA_POP_SIZE){
                Path p1 = tournamentSelection(population);
                Path p2 = tournamentSelection(population);

                // crossover
                Path c1, c2;
                if(this.random.nextDouble() < GA_CROSSOVER_RATE){
                    c1 = crossoverOX(p1, p2);
                    c2 = crossoverOX(p2, p1);
                } else {
                    c1 = copyOf(p1);
                    c2 = copyOf(p2);
                }

                // mutation
                mutateSwap(c1);
                mutateSwap(c2);

                // évaluation
                this.evaluation.evaluate(c1);
                this.evaluation.evaluate(c2);

                newPop.add(c1);
                if(newPop.size() < GA_POP_SIZE){
                    newPop.add(c2);
                }
            }

            population = newPop;

            // Mise à jour bestIndiv
            Path currentBest = getBest(population);
            double bestEval  = evaluation.quickEvaluate(bestIndiv);
            double currEval  = evaluation.quickEvaluate(currentBest);
            if(currEval < bestEval){
                bestIndiv = copyOf(currentBest);
            }
        }

        // Retourne le meilleur individu global
        return bestIndiv;
    }

    /**
     * Tournoi: on pioche GA_TOURNAMENT_SIZE individus, on garde le meilleur
     */
    private Path tournamentSelection(ArrayList<Path> pop){
        Path best = null;
        for(int i=0; i<GA_TOURNAMENT_SIZE; i++){
            Path candidate = pop.get(random.nextInt(pop.size()));
            if(best == null){
                best = candidate;
            } else {
                double be = evaluation.quickEvaluate(best);
                double ce = evaluation.quickEvaluate(candidate);
                if(ce < be) best = candidate;
            }
        }
        return best;
    }

    /**
     * Ordre crossover (OX)
     */
    private Path crossoverOX(Path p1, Path p2){
        int n = p1.getPath().length;
        int[] arr1 = p1.getPath();
        int[] arr2 = p2.getPath();

        int[] child = new int[n];
        for(int i=0; i<n; i++){
            child[i] = -1;
        }

        int start = random.nextInt(n);
        int end   = random.nextInt(n);
        if(start > end){
            int temp = start; start = end; end = temp;
        }

        // copie segment du parent1
        for(int i=start; i<=end; i++){
            child[i] = arr1[i];
        }

        // Remplir le reste depuis parent2
        int fillPos = (end+1) % n;
        for(int i=0; i<n; i++){
            int val = arr2[i];
            if(!contains(child, val)){
                child[fillPos] = val;
                fillPos = (fillPos+1) % n;
            }
        }
        return new Path(child);
    }

    /**
     * Mutation simple: swap de deux positions avec prob GA_MUTATION_RATE
     */
    private void mutateSwap(Path p){
        if(random.nextDouble() < GA_MUTATION_RATE){
            int[] arr = p.getCopyPath();
            int idx1 = random.nextInt(arr.length);
            int idx2 = random.nextInt(arr.length);
            int tmp = arr[idx1];
            arr[idx1] = arr[idx2];
            arr[idx2] = tmp;
            // on le remet
            for(int i=0; i<arr.length; i++){
                p.getPath()[i] = arr[i];
            }
        }
    }

    private boolean contains(int[] arr, int val){
        for(int x : arr){
            if(x == val) return true;
        }
        return false;
    }

    private Path getBest(ArrayList<Path> pop){
        // tri par évaluation croissante
        pop.sort(new Comparator<Path>(){
            @Override
            public int compare(Path o1, Path o2) {
                double e1 = evaluation.quickEvaluate(o1);
                double e2 = evaluation.quickEvaluate(o2);
                return Double.compare(e1, e2);
            }
        });
        return pop.get(0);
    }

    private Path copyOf(Path p){
        return new Path(p.getCopyPath());
    }

    private void reverseSegment(int[] route, int from, int to) {
        while (from<to){
            int temp = route[from];
            route[from] = route[to];
            route[to] = temp;
            from++;
            to--;
        }
    }
}
