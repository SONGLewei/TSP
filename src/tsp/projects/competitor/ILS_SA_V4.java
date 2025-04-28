package tsp.projects.competitor;

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.CompetitorProject;
import tsp.projects.InvalidProjectException;


public class ILS_SA_V4 extends CompetitorProject {


    private static final double COOLING_RATE = 0.999;
    private static final double RESTART_TEMPERATURE_FACTOR = 0.5;
    private static final int STAGNATION_LIMIT = 1200;
    private static final int LS_STRATEGY_N_THRESHOLD = 2000;
    private static final boolean USE_DOUBLE_BRIDGE_ON_STAGNATION = true;
    private static final boolean APPLY_LS_EVERY_ITERATION = false;

    private Random random;
    private Path currentPath;
    private double currentEvaluation;
    private Path bestPath;
    private double bestEvaluation;
    private double currentTemperature;
    private double initialTemperature;
    private double[][] distances;
    private int iterationsWithoutImprovement = 0;

    private static class MoveInfo {
        Path path;
        int i = -1;
        int j = -1;
        MoveInfo(Path p, int i, int j) { this.path = p; this.i = i; this.j = j; }
        MoveInfo(Path p) { this.path = p; }
    }

    public ILS_SA_V4(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SONG Lewei");
        this.setMethodName("PathfinderPhoenixV4");
        this.random = new Random();
    }

    @Override
    public void initialization() {
        int n = this.problem.getLength();
        precomputeDistances(n);

        Path initialLSResult;
        Path greedyPath = createGreedyPath(n);
        if(n>LS_STRATEGY_N_THRESHOLD){
            initialLSResult  = runFirstImprovement2Opt(greedyPath);
        }else {
            initialLSResult  = runFull2Opt(greedyPath);
        }
        this.currentPath = initialLSResult;
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.bestPath = new Path(this.currentPath.getCopyPath());
        this.bestEvaluation = this.currentEvaluation;

        this.initialTemperature = calculateInitialTemperatureHeuristic(n, this.distances);
        this.currentTemperature = this.initialTemperature;

        this.iterationsWithoutImprovement = 0;
    }

    @Override
    public void loop() {
        this.iterationsWithoutImprovement++;

        if (iterationsWithoutImprovement > STAGNATION_LIMIT) {

            Path perturbedPath;
            if (USE_DOUBLE_BRIDGE_ON_STAGNATION) {
                perturbedPath = applyDoubleBridgePerturbation(this.bestPath);
            } else {
                perturbedPath = applyMultipleSegmentReversals(this.bestPath, 3);
            }

            if(this.problem.getLength() > LS_STRATEGY_N_THRESHOLD){
                this.currentPath = runFirstImprovement2Opt(perturbedPath);
            }else {
                this.currentPath = runFull2Opt(perturbedPath);
            }

            this.currentEvaluation = this.evaluation.evaluate(this.currentPath);

            this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
            if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9;
            this.iterationsWithoutImprovement = 0;

            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
            }
            return;
        }

        Path currentPathBeforeMove = this.currentPath;
        int[] currentRouteArray = currentPathBeforeMove.getCopyPath();

        MoveInfo move = applySingleRandom2OptAndGetInfo(currentPathBeforeMove);
        Path neighborPath = move.path;
        double delta;

        if (move.i != -1) {
            int cityA = currentRouteArray[move.i];
            int cityB = currentRouteArray[(move.i + 1) % this.problem.getLength()];
            int cityC = currentRouteArray[move.j];
            int cityD = currentRouteArray[(move.j + 1) % this.problem.getLength()];

            double costRemoved = distances[cityA][cityB] + distances[cityC][cityD];
            double costAdded = distances[cityA][cityC] + distances[cityB][cityD];
            delta = costAdded - costRemoved;
        } else {
            delta = 0;
            neighborPath = currentPathBeforeMove;
        }

        Path pathBeforeLS;
        boolean acceptedMove = false;

        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            pathBeforeLS = neighborPath;
            acceptedMove = true;
        } else {
            pathBeforeLS = currentPathBeforeMove;
            acceptedMove = false;
        }

        Path refinedPath;
        double refinedEvaluation;

        if (acceptedMove) {

            if(this.problem.getLength() > LS_STRATEGY_N_THRESHOLD){
                refinedPath = runFirstImprovement2Opt(pathBeforeLS);
            }else {
                refinedPath = runFull2Opt(pathBeforeLS);
            }

            refinedEvaluation = this.evaluation.evaluate(refinedPath);
        } else {
            refinedPath = pathBeforeLS;
            refinedEvaluation = this.currentEvaluation;
        }

        this.currentPath = refinedPath;
        this.currentEvaluation = refinedEvaluation;

        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }

        coolDown();
    }

    private void updateBestSolution(Path path, double evaluation) {
        if (evaluation < this.bestEvaluation - 1e-9) {
            this.bestPath = new Path(path.getCopyPath());
            this.bestEvaluation = evaluation;
            this.evaluation.evaluate(this.bestPath);
            this.iterationsWithoutImprovement = 0;
        }
    }

    private Path runFull2Opt(Path initialPath) {
        int n = this.problem.getLength();
        int[] currentRoute = initialPath.getCopyPath();
        boolean improved = true;

        while (improved) {
            improved = false;
            double bestDelta = 0;
            int best_i = -1, best_j = -1;

            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    if (i == 0 && j == n - 1) continue;
                    int cityA = currentRoute[i];
                    int cityB = currentRoute[(i + 1) % n];
                    int cityC = currentRoute[j];
                    int cityD = currentRoute[(j + 1) % n];
                    double delta = (distances[cityA][cityC] + distances[cityB][cityD])
                            - (distances[cityA][cityB] + distances[cityC][cityD]);

                    if (delta < bestDelta - 1e-9) {
                        bestDelta = delta;
                        best_i = i;
                        best_j = j;
                        improved = true;
                    }
                }
            }

            if (improved) {
                reverseSegment(currentRoute, (best_i + 1) % n, best_j);
            }
        }
        return new Path(currentRoute);
    }

    private Path runFirstImprovement2Opt(Path initialPath) {
        int n = this.problem.getLength();
        int[] currentRoute = initialPath.getCopyPath();
        boolean improvedInPass = true;
        while (improvedInPass) {
            improvedInPass = false;
            find_first_improvement:
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    if (i == 0 && j == n - 1) continue;
                    int cityA = currentRoute[i]; int cityB = currentRoute[(i + 1) % n];
                    int cityC = currentRoute[j]; int cityD = currentRoute[(j + 1) % n];
                    double delta = (distances[cityA][cityC] + distances[cityB][cityD])
                            - (distances[cityA][cityB] + distances[cityC][cityD]);

                    if (delta < -1e-9) {
                        reverseSegment(currentRoute, (i + 1) % n, j);
                        improvedInPass = true;
                        break find_first_improvement;
                    }
                }
            }
        }
        return new Path(currentRoute);
    }

    private MoveInfo applySingleRandom2OptAndGetInfo(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 4) {
            return new MoveInfo(path);
        }

        int i, j;
        do {
            i = random.nextInt(n);
            j = random.nextInt(n);
            if (i == j) continue;
            if (i > j) { int temp = i; i = j; j = temp; }
        } while (j == i + 1 || (i == 0 && j == n - 1));

        reverseSegment(route, (i + 1) % n, j);
        return new MoveInfo(new Path(route), i, j);
    }

    private Path applyMultipleSegmentReversals(Path path, int numReversals) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 4) return path;
        for (int k = 0; k < numReversals; k++) {
            int i, j, attempts = 0;
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
                attempts++;
            } while ((j == i + 1 || (i == 0 && j == n - 1)) && attempts < 50);
            if (j == i + 1 || (i == 0 && j == n - 1)) {
                do { i = random.nextInt(n); j = random.nextInt(n); } while (i==j);
                if (i > j) { int temp = i; i = j; j = temp; }
            }
            reverseSegment(route, (i + 1) % n, j);
        }
        return new Path(route);
    }

    private Path applyDoubleBridgePerturbation(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 8) return applyMultipleSegmentReversals(path, 2);
        int i, j, k, l, attempts = 0;
        do {
            i = random.nextInt(n - 3); j = i + 1 + random.nextInt(Math.max(1, n - i - 3));
            k = j + 1 + random.nextInt(Math.max(1, n - j - 2)); l = k + 1 + random.nextInt(Math.max(1, n - k - 1));
            attempts++;
            if ((j > i + 1) && (k > j + 1) && (l > k + 1) || attempts > 50) break;
        } while (attempts <= 50);
        if (j <= i+1 || k <= j+1 || l <= k+1) return applyMultipleSegmentReversals(path, 2);
        reverseSegment(route, (i + 1) % n, j); reverseSegment(route, (k + 1) % n, l);
        return new Path(route);
    }

    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
        if (this.currentTemperature < 1e-12) this.currentTemperature = 1e-12;
    }
    private void precomputeDistances(int n) {
        this.distances = new double[n][n]; Coordinates[] coords = new Coordinates[n];
        for(int i=0; i<n; i++) coords[i] = this.problem.getCoordinates(i);
        for (int i = 0; i < n; i++) for (int j = i; j < n; j++) {
            if (i == j) this.distances[i][j] = 0;
            else { double dist = coords[i].distance(coords[j]); this.distances[i][j] = dist; this.distances[j][i] = dist; }
        }
    }
    private Path createGreedyPath(int n) {
        int[] route = new int[n]; boolean[] visited = new boolean[n]; int startNode = random.nextInt(n);
        route[0] = startNode; visited[startNode] = true; int currentCityIndex = startNode;
        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1; double minDistance = Double.POSITIVE_INFINITY;
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) if (!visited[neighborIndex]) {
                double distance = this.distances[currentCityIndex][neighborIndex];
                if (distance < minDistance) { minDistance = distance; nearestNeighbor = neighborIndex; }
            }
            if (nearestNeighbor == -1) { for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; } if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i)); }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true; currentCityIndex = nearestNeighbor;
        } return new Path(route);
    }
    private void reverseSegment(int[] route, int start, int end) {
        int n = route.length; if (n == 0) return; start = (start % n + n) % n; end = (end % n + n) % n;
        int segmentSize; if (start <= end) { segmentSize = end - start + 1; } else { segmentSize = (n - start) + end + 1; }
        int swaps = segmentSize / 2; int p1 = start; int p2 = end;
        for (int k = 0; k < swaps; k++) { int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp; p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n; }
    }
    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) {
        if (n <= 1) return 1.0; int samples = Math.min(n * (n - 1) / 2, 1000); double totalDistance = 0; int count = 0;
        if (n < 50) { for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) { totalDistance += distances[i][j]; count++; }
        } else { for (int k = 0; k < samples; k++) { int i = random.nextInt(n); int j; do { j = random.nextInt(n); } while (i == j); totalDistance += distances[i][j]; count++; } }
        double avgDistance = (count > 0) ? totalDistance / count : 100.0; double initialTemp = avgDistance * Math.sqrt(n) * 1.0;
        return Math.max(1.0, initialTemp);
    }
}