package tsp.projects.versionsNoUse;

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

public class SA_Swap2Opt_LS_Restart_V2 extends DemoProject { // Renamed V3

    // --- Parameters ---
    private static final double COOLING_RATE = 0.999; // SA component cooling rate
    private static final double RESTART_TEMPERATURE_FACTOR = 0.5; // Restart temperature factor (potentially higher for ILS)
    private static final int STAGNATION_LIMIT = 500; // Iterations without best improvement for major restart
    private static final boolean USE_DOUBLE_BRIDGE_ON_STAGNATION = true;
    // NEW/MODIFIED for ILS-SA:
    private static final double SA_ACCEPTANCE_PROBABILITY_FLOOR = 0.01; // Minimum acceptance probability for SA part (optional)
    private static final boolean APPLY_LS_EVERY_ITERATION = false; // Apply Local Search frequently

    // --- State Variables ---
    private Random random;
    private Path currentPath;        // Current base path for next iteration (should be locally optimal)
    private double currentEvaluation;
    private Path bestPath;           // Global best path found
    private double bestEvaluation;
    private double currentTemperature; // Temperature for SA acceptance criterion
    private double initialTemperature;
    private double[][] distances;
    private int iterationsWithoutImprovement = 0; // Stagnation counter

    public SA_Swap2Opt_LS_Restart_V2(Evaluation evaluation) throws InvalidProjectException { // V3 Constructor
        super(evaluation);
        this.addAuthor("SONG Lewei (V3 - ILS Hybrid)"); // V3 Author
        this.setMethodName("PathfinderPhoenixV3"); // V3 Method Name
        this.random = new Random();
    }

    @Override
    public void initialization() {

        this.iterationsWithoutImprovement = 0;

        int n = this.problem.getLength();
        precomputeDistances(n);

        // Initial path generation and Local Search
        Path greedyPath = createGreedyPath(n);
        this.currentPath = runFull2Opt(greedyPath); // Start with a 2-opt optimal path
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.bestPath = new Path(this.currentPath); // Use Path constructor for deep copy
        this.bestEvaluation = this.currentEvaluation;

        // Temperature initialization for SA acceptance part
        this.initialTemperature = calculateInitialTemperatureHeuristic(n, this.distances);
        this.currentTemperature = this.initialTemperature;

        this.iterationsWithoutImprovement = 0;
    }

    @Override
    public void loop() {
        iterationsWithoutImprovement++;

        // --- 1. Check for Stagnation: Major Perturbation & Restart ---

        if (iterationsWithoutImprovement > STAGNATION_LIMIT) {

            // Perturb the *best known* path strongly
            Path perturbedPath;
            if (USE_DOUBLE_BRIDGE_ON_STAGNATION) {
                perturbedPath = applyDoubleBridgePerturbation(this.bestPath);
            } else {
                perturbedPath = applyMultipleSegmentReversals(this.bestPath, 3); // Example: 3 reversals
            }

            // Apply Local Search to the perturbed path
            this.currentPath = runFull2Opt(perturbedPath);
            this.currentEvaluation = this.evaluation.evaluate(this.currentPath);

            // Reset temperature and counter
            this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
            if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9;
            this.iterationsWithoutImprovement = 0;

            // Update best if the new locally optimal path is better
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
            }
            return; // Skip the rest of the loop for this iteration
        }

        // --- 2. Generate Candidate via SA-like step (optional, can be simple neighbor) ---
        // Generate a neighbor (e.g., one 2-opt or swap move) from the current path
        Path neighborPath;
        double delta; // Cost change from currentPath to neighborPath before LS

        // Simple approach: just make one random 2-opt move as a candidate generator
        neighborPath = applySingleRandom2Opt(currentPath);
        // Evaluate difference (can be approximate or exact)
        double neighborEval = this.evaluation.evaluate(neighborPath); // Evaluate fully for simplicity here
        delta = neighborEval - this.currentEvaluation;

        // --- 3. SA Acceptance or Selection ---
        Path pathBeforeLS;
        double evalBeforeLS;

        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            // Accept the neighbor move (even if worse, probabilistically)
            pathBeforeLS = neighborPath;
            evalBeforeLS = neighborEval;
            // Optional: Log acceptance of worsening move
            // if (delta > 0) System.out.printf("Accepted worse move (Delta: %.2f, Prob: %.4f)%n", delta, Math.exp(-delta / currentTemperature));

        } else {
            // Reject the neighbor move, stay with the current path for LS step
            pathBeforeLS = this.currentPath;
            evalBeforeLS = this.currentEvaluation;
        }

        // --- 4. Apply Local Search ---
        Path refinedPath;
        double refinedEvaluation;
        if (APPLY_LS_EVERY_ITERATION || pathBeforeLS != this.currentPath) { // Apply LS if accepted move or always
            refinedPath = runFull2Opt(pathBeforeLS);
            refinedEvaluation = this.evaluation.evaluate(refinedPath);
        } else { // Optionally skip LS if move was rejected and LS is not forced every iter
            refinedPath = pathBeforeLS; // which is currentPath
            refinedEvaluation = evalBeforeLS; // which is currentEvaluation
        }


        // --- 5. Update Current State ---
        this.currentPath = refinedPath;
        this.currentEvaluation = refinedEvaluation;

        // --- 6. Update Best Solution Found ---
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }

        // --- 7. Cool Down Temperature for SA Acceptance ---
        coolDown();
    }

    // MODIFIED: Reset stagnation counter when best solution is updated
    private void updateBestSolution(Path path, double evaluation) {
        // Use tolerance for float comparison
        if (evaluation < this.bestEvaluation - 1e-9) {
            // Make a defensive copy for bestPath
            this.bestPath = new Path(path.getCopyPath());
            this.bestEvaluation = evaluation;
            // Ensure the framework's best path is updated if necessary
            this.evaluation.evaluate(this.bestPath);
            this.iterationsWithoutImprovement = 0; // Reset counter
        }
    }


    // --- Local Search ---
    // NEW: Full 2-Opt Local Search (Best Improvement)
    private Path runFull2Opt(Path initialPath) {
        int n = this.problem.getLength();
        int[] currentRoute = initialPath.getCopyPath();
        double currentEval = this.evaluation.evaluate(initialPath); // Get initial cost
        boolean improved = true;

        while (improved) {
            improved = false;
            double bestDelta = 0;
            int best_i = -1, best_j = -1;

            // Find the best possible 2-opt move in the neighborhood
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    // Skip wrap-around move
                    if (i == 0 && j == n - 1) continue;

                    // Calculate delta for this potential 2-opt move
                    int cityA = currentRoute[i];
                    int cityB = currentRoute[(i + 1) % n];
                    int cityC = currentRoute[j];
                    int cityD = currentRoute[(j + 1) % n];

                    double delta = (distances[cityA][cityC] + distances[cityB][cityD])
                            - (distances[cityA][cityB] + distances[cityC][cityD]);

                    // If this move is better than the best found so far in this pass
                    if (delta < bestDelta - 1e-9) { // Use tolerance
                        bestDelta = delta;
                        best_i = i;
                        best_j = j;
                        improved = true; // Mark that potential improvement exists
                    }
                }
            }

            // If an improving move was found, apply it
            if (improved) {
                reverseSegment(currentRoute, (best_i + 1) % n, best_j);
                currentEval += bestDelta; // Update cost incrementally
                // Continue the while loop to check for further improvements from the new path
            }
            // If no improving move found in the entire neighborhood, exit while loop
        }
        // Return the 2-opt optimal path found
        Path finalPath = new Path(currentRoute);
        // Verify evaluation matches incremental update (optional debug)
        // double finalEval = this.evaluation.evaluate(finalPath);
        // if (Math.abs(finalEval - currentEval) > 1e-6) System.err.println("Warning: 2-Opt Eval mismatch!");
        return finalPath;
    }

    // --- Perturbation Methods ---

    // Applies a single random 2-opt move
    private Path applySingleRandom2Opt(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 4) return path;

        int i, j;
        do {
            i = random.nextInt(n);
            j = random.nextInt(n);
            if (i == j) continue;
            if (i > j) { int temp = i; i = j; j = temp; }
        } while (j == i + 1 || (i == 0 && j == n - 1));

        reverseSegment(route, (i + 1) % n, j);
        return new Path(route);
    }

    // NEW: Apply multiple segment reversals
    private Path applyMultipleSegmentReversals(Path path, int numReversals) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 4) return path;

        for (int k = 0; k < numReversals; k++) {
            int i, j;
            int attempts = 0;
            // Try to find non-adjacent pair
            do {
                i = random.nextInt(n);
                j = random.nextInt(n);
                if (i == j) continue;
                if (i > j) { int temp = i; i = j; j = temp; }
                attempts++;
            } while ((j == i + 1 || (i == 0 && j == n - 1)) && attempts < 50);
            // Fallback: just pick any two distinct if non-adjacent fails
            if (j == i + 1 || (i == 0 && j == n - 1)) {
                do { i = random.nextInt(n); j = random.nextInt(n); } while (i==j);
                if (i > j) { int temp = i; i = j; j = temp; }
            }
            reverseSegment(route, (i + 1) % n, j);
        }
        return new Path(route);
    }

    // Double Bridge (same as V2)
    private Path applyDoubleBridgePerturbation(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();
        if (n < 8) {
            return applyMultipleSegmentReversals(path, 2); // Fallback for small N
        }
        int i, j, k, l;
        int attempts = 0;
        do {
            i = random.nextInt(n - 3);
            j = i + 1 + random.nextInt(Math.max(1, n - i - 3)); // Ensure j > i, handle edge case n-i-3 <= 0
            k = j + 1 + random.nextInt(Math.max(1, n - j - 2)); // Ensure k > j
            l = k + 1 + random.nextInt(Math.max(1, n - k - 1)); // Ensure l > k
            attempts++;
            boolean nonTrivial = (j > i + 1) && (k > j + 1) && (l > k + 1);
            if (nonTrivial || attempts > 50) break;
        } while (attempts <= 50);

        if (j <= i+1 || k <= j+1 || l <= k+1) { // Fallback if failed
            return applyMultipleSegmentReversals(path, 2);
        }
        reverseSegment(route, (i + 1) % n, j);
        reverseSegment(route, (k + 1) % n, l);
        return new Path(route);
    }

    // --- Other Helper Methods (Mostly Unchanged) ---

    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
        // Add a stricter lower bound floor for temperature
        if (this.currentTemperature < 1e-12) this.currentTemperature = 1e-12;
    }

    private void precomputeDistances(int n) {
        // ... (same as V2) ...
        this.distances = new double[n][n];
        Coordinates[] coords = new Coordinates[n];
        for(int i=0; i<n; i++) {
            coords[i] = this.problem.getCoordinates(i);
        }

        for (int i = 0; i < n; i++) {
            for (int j = i; j < n; j++) { // Iterate j from i
                if (i == j) {
                    this.distances[i][j] = 0;
                } else {
                    double dist = coords[i].distance(coords[j]);
                    this.distances[i][j] = dist;
                    this.distances[j][i] = dist; // Symmetric distance
                }
            }
        }
    }

    private Path createGreedyPath(int n) {
        // ... (same as V2) ...
        int[] route = new int[n];
        boolean[] visited = new boolean[n];
        int startNode = random.nextInt(n);
        route[0] = startNode;
        visited[startNode] = true;
        int currentCityIndex = startNode;

        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1;
            double minDistance = Double.POSITIVE_INFINITY;
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) {
                if (!visited[neighborIndex]) {
                    double distance = this.distances[currentCityIndex][neighborIndex];
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestNeighbor = neighborIndex;
                    }
                }
            }
            if (nearestNeighbor == -1) {
                for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i));
            }
            route[i] = nearestNeighbor;
            visited[nearestNeighbor] = true;
            currentCityIndex = nearestNeighbor;
        }
        return new Path(route);
    }

    private void reverseSegment(int[] route, int start, int end) {
        // ... (same as V2) ...
        int n = route.length;
        if (n == 0) return;
        start = (start % n + n) % n;
        end = (end % n + n) % n;
        int segmentSize;
        if (start <= end) { segmentSize = end - start + 1; }
        else { segmentSize = (n - start) + end + 1; }
        int swaps = segmentSize / 2;
        int p1 = start; int p2 = end;
        for (int k = 0; k < swaps; k++) {
            int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp;
            p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n;
        }
    }

    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) {
        // ... (same as V2, maybe adjust factor if needed) ...
        if (n <= 1) return 1.0;
        int samples = Math.min(n * (n - 1) / 2, 1000);
        double totalDistance = 0; int count = 0;
        if (n < 50) {
            for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) { totalDistance += distances[i][j]; count++; }
        } else {
            for (int k = 0; k < samples; k++) {
                int i = random.nextInt(n); int j; do { j = random.nextInt(n); } while (i == j);
                totalDistance += distances[i][j]; count++;
            }
        }
        double avgDistance = (count > 0) ? totalDistance / count : 100.0;
        double initialTemp = avgDistance * Math.sqrt(n) * 1.0; // Factor might need tuning for ILS-SA
        return Math.max(1.0, initialTemp);
    }

    // Need calculateSwapDelta if used in neighbor generation (not used in current V3 loop)
    // Need performSAIterationWithMixedNeighborhood if used (not used in current V3 loop)
    // Need performIntensive2OptOnBestPath if used (replaced by runFull2Opt)
}