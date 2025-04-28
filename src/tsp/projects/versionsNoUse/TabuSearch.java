package tsp.projects.versionsNoUse;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.CompetitorProject;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

public class TabuSearch extends DemoProject {
    private Random random;
    private Path bestRoute;
    private Path currentRoute;
    private boolean finished = false;
    private long startTime;
    
    // Tabu Search parameters
    private Queue<int[]> tabuList;
    private HashSet<String> tabuSet;
    private int tabuTenure;
    private int iterationsWithoutImprovement;
    private final int MAX_RUNTIME_MS = 5000;
    
    // Adaptive search parameters
    private double bestEvaluation;
    private int searchPhase; // 0=intensification, 1=diversification
    private double temperature; // For simulated annealing component
    private int[][] frequencyMatrix; // For long-term memory

    public TabuSearch(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("Lewei SONG ");
        this.addAuthor(" SAGANG TANWOUO Achille ");
        this.setMethodName(" Tabu Search ");
    }

    @Override
    public void initialization() {
        this.startTime = System.currentTimeMillis();
        this.random = new Random();
        int n = this.problem.getLength();
        
        // Initialize adaptive parameters
        this.tabuTenure = (int)(n * 0.3); // Dynamic tabu tenure
        this.tabuList = new LinkedList<>();
        this.tabuSet = new HashSet<>();
        this.temperature = n * 10; // Initial temperature
        this.searchPhase = 0;
        this.frequencyMatrix = new int[n][n];
        
        // Create initial solution with randomized greedy + 2-opt kick
        this.currentRoute = generateInitialSolution(n);
        this.bestRoute = new Path(this.currentRoute.getPath().clone());
        this.bestEvaluation = this.evaluation.evaluate(this.currentRoute);
        this.iterationsWithoutImprovement = 0;
    }

    @Override
    public void loop() {

        // Adaptive phase control
        double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        if (elapsed > 45) { // Last 15 seconds - intensification
            searchPhase = 2;
        } else if (iterationsWithoutImprovement > 50) {
            searchPhase = 1; // Diversification
        } else {
            searchPhase = 0; // Normal search
        }

        switch (searchPhase) {
            case 0: // Intensification phase
                tabuSearchStep();
                break;
            case 1: // Diversification phase
                diversify();
                iterationsWithoutImprovement = 0;
                break;
            case 2: // Final intensification phase
                aggressiveIntensification();
                break;
        }
        
        // Cooling schedule
        this.temperature *= 0.995;
    }

    private void tabuSearchStep() {
        int n = this.problem.getLength();
        int[] currentPath = this.currentRoute.getPath();
        
        // Candidate list of moves
        int candidateSize = Math.min(200, n * 2);
        int[][] candidates = new int[candidateSize][2];
        double[] candidateValues = new double[candidateSize];
        
        // Generate candidate moves
        for (int i = 0; i < candidateSize; i++) {
            int a = random.nextInt(n - 1);
            int b = a + 2 + random.nextInt(n - a - 2);
            if (a == 0 && b == n - 1) {
                i--; // Skip invalid move
                continue;
            }
            candidates[i][0] = a;
            candidates[i][1] = b;
            candidateValues[i] = evaluateMove(currentPath, a, b);
        }
        
        // Find best admissible move
        int bestA = -1, bestB = -1;
        double bestValue = Double.POSITIVE_INFINITY;
        for (int i = 0; i < candidateSize; i++) {
            int a = candidates[i][0];
            int b = candidates[i][1];
            String moveSig = createMoveSignature(currentPath, a, b);
            
            if ((!tabuSet.contains(moveSig) || candidateValues[i] < bestEvaluation * 0.1000) 
                && candidateValues[i] < bestValue) {
                bestValue = candidateValues[i];
                bestA = a;
                bestB = b;
            }
        }
        
        // Apply the best move found
        if (bestA != -1) {
            applyMove(currentPath, bestA, bestB, bestValue);
        } else {
            iterationsWithoutImprovement++;
        }
    }

    private void aggressiveIntensification() {
        int n = this.problem.getLength();
        int[] currentPath = this.currentRoute.getPath();
        boolean improved = false;
        
        // Limited exhaustive 2-opt search around best solution
        for (int i = 0; i < n - 1 && !improved; i++) {
            for (int j = i + 2; j < n && !improved; j++) {
                if (i == 0 && j == n - 1) continue;
                
                double moveValue = evaluateMove(currentPath, i, j);
                if (moveValue < bestEvaluation) {
                    applyMove(currentPath, i, j, moveValue);
                    improved = true;
                }
            }
        }
        
        if (!improved) {
            iterationsWithoutImprovement++;
        }
    }

    private void diversify() {
        int n = this.problem.getLength();
        int[] path = this.currentRoute.getPath().clone();
        
        // Double-bridge kick (4-opt move)
        int pos1 = 1 + random.nextInt(n / 4);
        int pos2 = pos1 + 1 + random.nextInt(n / 4);
        int pos3 = pos2 + 1 + random.nextInt(n / 4);
        
        // Perform the double bridge move
        int[] newPath = new int[n];
        System.arraycopy(path, 0, newPath, 0, pos1);
        System.arraycopy(path, pos3, newPath, pos1, n - pos3);
        System.arraycopy(path, pos2, newPath, pos1 + n - pos3, pos3 - pos2);
        System.arraycopy(path, pos1, newPath, pos1 + n - pos3 + pos3 - pos2, pos2 - pos1);
        
        // Update frequency matrix
        updateFrequencyMatrix(path, newPath);
        
        this.currentRoute = new Path(newPath);
        double newEval = this.evaluation.evaluate(this.currentRoute);
        
        // Update best if improved
        if (newEval < bestEvaluation) {
            this.bestRoute = new Path(newPath.clone());
            this.bestEvaluation = newEval;
        }
        
        // Reset tabu list
        this.tabuList.clear();
        this.tabuSet.clear();
    }

    private Path generateInitialSolution(int n) {
        // Randomized greedy construction with 2-opt kick
        int[] route = greedyConstruction(n);
        
        // Apply 3 random 2-opt moves to diversify
        for (int k = 0; k < 3; k++) {
            int i = random.nextInt(n - 1);
            int j = i + 2 + random.nextInt(n - i - 2);
            reverse(route, i + 1, j);
        }
        
        return new Path(route);
    }

    private int[] greedyConstruction(int n) {
        int[] route = new int[n];
        boolean[] visited = new boolean[n];
        int current = random.nextInt(n);
        route[0] = current;
        visited[current] = true;
        
        for (int i = 1; i < n; i++) {
            // Find k nearest neighbors
            int k = Math.min(5, n - i);
            int[] candidates = new int[k];
            double[] distances = new double[k];
            
            for (int j = 0, found = 0; j < n && found < k; j++) {
                if (!visited[j]) {
                    double dist = this.problem.getCoordinates(current)
                                   .distance(this.problem.getCoordinates(j));
                    // Insert sorted
                    int pos = found;
                    while (pos > 0 && dist < distances[pos-1]) {
                        if (pos < k) {
                            candidates[pos] = candidates[pos-1];
                            distances[pos] = distances[pos-1];
                        }
                        pos--;
                    }
                    if (pos < k) {
                        candidates[pos] = j;
                        distances[pos] = dist;
                        found++;
                    }
                }
            }
            
            // Probabilistic selection
            int next;
            if (random.nextDouble() < 0.7) {
                next = candidates[0];
            } else {
                next = candidates[random.nextInt(Math.min(3, k))];
            }
            
            route[i] = next;
            visited[next] = true;
            current = next;
        }
        return route;
    }

    private double evaluateMove(int[] path, int i, int j) {
        int n = path.length;
        Coordinates ci = this.problem.getCoordinates(path[i]);
        Coordinates ci1 = this.problem.getCoordinates(path[(i+1)%n]);
        Coordinates cj = this.problem.getCoordinates(path[j]);
        Coordinates cj1 = this.problem.getCoordinates(path[(j+1)%n]);
        
        double current = ci.distance(ci1) + cj.distance(cj1);
        double proposed = ci.distance(cj) + ci1.distance(cj1);
        
        // Consider frequency penalty
        double penalty = 0.01 * frequencyMatrix[path[i]][path[j]];
        return proposed - current + penalty;
    }

    private void applyMove(int[] path, int i, int j, double moveValue) {
        int[] newPath = path.clone();
        reverse(newPath, i + 1, j);
        
        // Update tabu list
        String moveSig = createMoveSignature(path, i, j);
        tabuSet.add(moveSig);
        tabuList.add(new int[]{i, j});
        if (tabuList.size() > tabuTenure) {
            int[] oldMove = tabuList.poll();
            tabuSet.remove(createMoveSignatureFromIndices(oldMove[0], oldMove[1]));
        }
        
        // Update solution
        this.currentRoute = new Path(newPath);
        if (moveValue < bestEvaluation) {
            this.bestRoute = new Path(newPath.clone());
            this.bestEvaluation = moveValue;
            this.iterationsWithoutImprovement = 0;
        } else {
            this.iterationsWithoutImprovement++;
        }
        
        // Update frequency matrix
        frequencyMatrix[path[i]][path[j]]++;
        frequencyMatrix[path[j]][path[i]]++;
    }

    private void updateFrequencyMatrix(int[] oldPath, int[] newPath) {
        // Penalize edges that are being removed
        int n = oldPath.length;
        for (int i = 0; i < n; i++) {
            int a = oldPath[i];
            int b = oldPath[(i+1)%n];
            frequencyMatrix[a][b]++;
            frequencyMatrix[b][a]++;
        }
    }

    private void reverse(int[] route, int from, int to) {
        while (from < to) {
            int temp = route[from];
            route[from] = route[to];
            route[to] = temp;
            from++;
            to--;
        }
    }
    
    private String createMoveSignature(int[] path, int i, int j) {
        return path[i] + "-" + path[(i+1)%path.length] + "-" + path[j] + "-" + path[(j+1)%path.length];
    }
    
    private String createMoveSignatureFromIndices(int i, int j) {
        int[] path = this.currentRoute.getPath();
        return path[i] + "-" + path[(i+1)%path.length] + "-" + path[j] + "-" + path[(j+1)%path.length];
    }
}