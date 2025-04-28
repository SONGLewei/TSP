package tsp.projects.versionsNoUse;

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;


public class SA_Swap2Opt_LS_Restart extends DemoProject {

    private static final double COOLING_RATE = 0.999;///998
    private static final double MIN_TEMPERATURE_FACTOR = 0.005;
    private static final double RESTART_TEMPERATURE_FACTOR = 0.05;//0.05
    private static final double MIN_PERTURBATION_SEGMENT_RATIO = 0.4;
    private static final double MAX_PERTURBATION_SEGMENT_RATIO = 0.5;
    private static final double SWAP_PROBABILITY = 0.1;

    private Random random;
    private Path currentPath;
    private double currentEvaluation;
    private Path bestPath;
    private double bestEvaluation;
    private double currentTemperature;
    private double initialTemperature;
    private double minTemperature;
    private double[][] distances;
    private boolean intensiveLocalSearchPhase = false;
    private boolean bestPathIs2OptOptimal = false;

    public SA_Swap2Opt_LS_Restart(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SONG Lewei");
        this.setMethodName("PathfinderPhoenix");
        this.random = new Random();
    }

    @Override
    public void initialization() {
        int n = this.problem.getLength();
        precomputeDistances(n);
        this.currentPath = createGreedyPath(n);
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.bestPath = new Path(this.currentPath);
        this.bestEvaluation = this.currentEvaluation;

        this.initialTemperature = calculateInitialTemperatureHeuristic(n, this.distances);
        this.minTemperature = this.initialTemperature * MIN_TEMPERATURE_FACTOR;
        if (this.minTemperature < 1e-6) this.minTemperature = 1e-6;
        this.currentTemperature = this.initialTemperature;

        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false;

        System.out.println("SA_Swap2Opt_LS_Restart Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f, Swap Prob: %.2f%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR, SWAP_PROBABILITY);
    }

    @Override
    public void loop() {
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= this.minTemperature) {
            this.intensiveLocalSearchPhase = true;
            this.bestPathIs2OptOptimal = false;
        }

        if (this.intensiveLocalSearchPhase) {
            if (!this.bestPathIs2OptOptimal) {
                performIntensive2OptOnBestPath();
            } else {
                restartSearchWithLargePerturbation();
            }
        } else {
            performSAIterationWithMixedNeighborhood();
            coolDown();
        }


        }

    private void restartSearchWithLargePerturbation() {
        Path perturbedPath = applyLargeSegmentReversalPerturbation(this.bestPath);
        this.currentPath = perturbedPath;
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
        if (this.currentTemperature < this.minTemperature) this.currentTemperature = this.minTemperature * 1.1;
        if (this.currentTemperature <= 0) this.currentTemperature = 1e-6;
        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false;
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }
    }
    private Path applyLargeSegmentReversalPerturbation(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 4) return path;
        int minLen = (int) Math.max(2, n * MIN_PERTURBATION_SEGMENT_RATIO);
        int maxLen = (int) Math.min(n - 2, n * MAX_PERTURBATION_SEGMENT_RATIO);
        if (maxLen <= minLen) { minLen = 2; maxLen = n - 2;}
        int i = -1, j = -1, attempts = 0;
        while(attempts < 100) {
            int idx1 = random.nextInt(n); int idx2 = random.nextInt(n);
            if (idx1 == idx2) { attempts++; continue; }
            if (idx1 > idx2) { int temp = idx1; idx1 = idx2; idx2 = temp; }
            int segmentLength = idx2 - idx1; int wrapAroundLength = n - segmentLength;
            if ((segmentLength >= minLen && segmentLength <= maxLen) || (wrapAroundLength >= minLen && wrapAroundLength <= maxLen)) {
                i = idx1; j = idx2; break;
            }
            attempts++;
        }
        if (i == -1 || j == -1 || (i == 0 && j == n - 1) || j == i+1) {
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1));
        }
        reverseSegment(route, (i + 1) % n, j);
        return new Path(route);
    }


    private void performSAIterationWithMixedNeighborhood() {
        Path neighborPath;
        double delta;
        int[] currentRouteArray = this.currentPath.getCopyPath();

        if (random.nextDouble() < SWAP_PROBABILITY) {
            int n = this.problem.getLength();
            if (n < 2) return;
            int i = random.nextInt(n);
            int j;
            do {
                j = random.nextInt(n);
            } while (i == j);

            delta = calculateSwapDelta(currentRouteArray, i, j);

            int temp = currentRouteArray[i];
            currentRouteArray[i] = currentRouteArray[j];
            currentRouteArray[j] = temp;
            neighborPath = new Path(currentRouteArray);

        } else {
            int n = this.problem.getLength();
            if (n < 4) return;
            int i, j;
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1));

            int cityA_idx = currentRouteArray[i];
            int cityB_idx = currentRouteArray[(i + 1) % n];
            int cityC_idx = currentRouteArray[j];
            int cityD_idx = currentRouteArray[(j + 1) % n];
            double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
            double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
            delta = costAdded - costRemoved;

            reverseSegment(currentRouteArray, (i + 1) % n, j);
            neighborPath = new Path(currentRouteArray);
        }

        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            this.currentPath = neighborPath;
            this.currentEvaluation += delta;

            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
                this.bestPathIs2OptOptimal = false;
            }
        }
    }


    private double calculateSwapDelta(int[] route, int i, int j) {
        int n = route.length;
        int city_i = route[i];
        int city_j = route[j];
        int city_pi = route[(i - 1 + n) % n];
        int city_ni = route[(i + 1) % n];
        int city_pj = route[(j - 1 + n) % n];
        int city_nj = route[(j + 1) % n];

        double delta = 0;

        if ((i + 1) % n == j) {
            delta -= distances[city_pi][city_i];
            delta -= distances[city_i][city_j];
            delta -= distances[city_j][city_nj];
            delta += distances[city_pi][city_j];
            delta += distances[city_j][city_i];
            delta += distances[city_i][city_nj];
        } else if ((j + 1) % n == i) {
            delta -= distances[city_pj][city_j];
            delta -= distances[city_j][city_i];
            delta -= distances[city_i][city_ni];
            delta += distances[city_pj][city_i];
            delta += distances[city_i][city_j];
            delta += distances[city_j][city_ni];
        } else {
            delta -= distances[city_pi][city_i];
            delta -= distances[city_i][city_ni];
            delta -= distances[city_pj][city_j];
            delta -= distances[city_j][city_nj];
            delta += distances[city_pi][city_j];
            delta += distances[city_j][city_ni];
            delta += distances[city_pj][city_i];
            delta += distances[city_i][city_nj];
        }

        return delta;
    }


    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
        if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9;
    }
    private void updateBestSolution(Path path, double evaluation) {
        if (evaluation < this.bestEvaluation - 1e-9) {
            this.bestPath = path; this.bestEvaluation = evaluation; this.evaluation.evaluate(this.bestPath);
        } else if (Math.abs(evaluation - this.bestEvaluation) < 1e-9) {
            this.bestEvaluation = evaluation; this.evaluation.evaluate(path);
        } else { this.evaluation.evaluate(this.bestPath); }
    }

    private void performIntensive2OptOnBestPath() {
        int n = this.problem.getLength(); int[] route = this.bestPath.getCopyPath();
        double currentBestEval = this.bestEvaluation; boolean improvedInThisStep = false;
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 2; j < n; j++) {
                if (i == 0 && j == n - 1) continue;
                int cityA_idx = route[i]; int cityB_idx = route[(i + 1) % n];
                int cityC_idx = route[j]; int cityD_idx = route[(j + 1) % n];
                double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
                double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
                double delta = costAdded - costRemoved;
                if (delta < -1e-9) {
                    reverseSegment(route, (i + 1) % n, j); Path improvedPath = new Path(route);
                    double newEvaluation = currentBestEval + delta; updateBestSolution(improvedPath, newEvaluation);
                    improvedInThisStep = true; return;
                } } }
        if (!improvedInThisStep) { this.bestPathIs2OptOptimal = true; }
    }

    private void precomputeDistances(int n) {
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) {
            Coordinates c1 = this.problem.getCoordinates(i);
            for (int j = i; j < n; j++) {
                if (i == j) this.distances[i][j] = 0;
                else {
                    Coordinates c2 = this.problem.getCoordinates(j);
                    double dist = c1.distance(c2);
                    this.distances[i][j] = dist; this.distances[j][i] = dist;
                } } }
    }
    private Path createGreedyPath(int n) {
        int[] route = new int[n]; boolean[] visited = new boolean[n];
        int startNode = random.nextInt(n); route[0] = startNode; visited[startNode] = true;
        int currentCityIndex = startNode;
        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1; double minDistance = Double.POSITIVE_INFINITY;
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) {
                if (!visited[neighborIndex]) {
                    double distance = this.distances[currentCityIndex][neighborIndex];
                    if (distance < minDistance) { minDistance = distance; nearestNeighbor = neighborIndex; }
                } }
            if (nearestNeighbor == -1) {
                for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i));
            }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true; currentCityIndex = nearestNeighbor;
        } return new Path(route);
    }

    private void reverseSegment(int[] route, int start, int end) {
        int n = route.length; start = start % n; end = end % n;
        if (start == end) return;
        int p1 = start; int p2 = end; int segmentSize;
        if (p1 < p2) { segmentSize = (p2 - p1 + 1) / 2; }
        else { segmentSize = (n - p1 + p2 + 1) / 2; }
        for (int k = 0; k < segmentSize; k++) {
            int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp;
            p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n;
        }
    }

    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) {
        double avgDistance = 0;
        int count = 0;
        for (int i = 0; i < Math.min(n, 100); i++) {
            for (int j = i + 1; j < Math.min(n, 100); j++) {
                if (i < n && j < n) {
                    avgDistance += distances[i][j];
                    count++;
                }
            }
        }
        if (count > 0) avgDistance /= count;
        else avgDistance = 100;
        double initialTemp = avgDistance * n * 1;
        return Math.max(1.0, initialTemp);
    }
}