package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 结合 SA, Swap/2-Opt 邻域, 后期 2-opt 局部搜索, 以及 Swap变异 重启机制。
 * (修复版 v2: 显著降低 Swap 变异强度)
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on SA_Swap2Opt_LS_SwapRestart (Bugfixed v2)
 */
public class SA_Swap20pt_LS_SwapRestart extends DemoProject { // 新名字 v2

    // --- 参数 (需要调优!) ---
    private static final double COOLING_RATE = 0.998;          // 冷却率
    private static final double MIN_TEMPERATURE_FACTOR = 0.005;    // 触发最终 LS/重启的温度因子
    private static final double RESTART_TEMPERATURE_FACTOR = 0.5; // 重启温度因子 (保持较低)
    // **** 修改点: 大幅降低扰动强度 ****
    private static final int    MUTATION_SWAPS = 15; // 重启/变异时执行的 Swap 次数 (显著减少! 可调: 5-30)
    private static final double SWAP_PROBABILITY = 0.4;          // SA阶段使用Swap的概率

    // --- 内部状态 ---
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
    private boolean bestPathIsLocallyOptimal = false;
    private static final double EPSILON = 1e-9;

    /** 构造函数 */
    public SA_Swap20pt_LS_SwapRestart(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SAGANG TANWOUO Achille");
        this.addAuthor("SONG Lewei");
        this.setMethodName("SA_Swap2Opt_LS_SwapRestart_v2"); // 新名字 v2
        this.random = new Random();
    }

    /** 初始化 */
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
        if (this.minTemperature < EPSILON) this.minTemperature = EPSILON;
        this.currentTemperature = this.initialTemperature;

        this.intensiveLocalSearchPhase = false;
        this.bestPathIsLocallyOptimal = false;

        System.out.println("SA_Swap2Opt_LS_SwapRestart_v2 Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f, Swap Prob: %.2f, Mutation Swaps: %d%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR, SWAP_PROBABILITY, MUTATION_SWAPS);
    }

    /** 主循环 */
    @Override
    public void loop() {
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= this.minTemperature) {
            this.intensiveLocalSearchPhase = true;
            this.bestPathIsLocallyOptimal = false;
        }

        if (this.intensiveLocalSearchPhase) {
            if (!this.bestPathIsLocallyOptimal) {
                performIntensive2OptOnBestPath();
            } else {
                restartSearchWithSwapMutation(); // 使用 Swap 进行扰动/变异并重启
            }
        } else {
            performSAIterationWithMixedNeighborhood();
            coolDown();
        }
    }

    // --- 重启逻辑 (使用 Swap Mutation) ---
    private void restartSearchWithSwapMutation() {
        Path mutatedPath = applySwapMutation(this.bestPath, MUTATION_SWAPS); // 使用 MUTATION_SWAPS
        this.currentPath = mutatedPath;
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
        if (this.currentTemperature < this.minTemperature) this.currentTemperature = this.minTemperature * 1.1;
        if (this.currentTemperature <= 0) this.currentTemperature = EPSILON;
        this.intensiveLocalSearchPhase = false;
        this.bestPathIsLocallyOptimal = false;
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }
    }

    // --- 其他方法保持不变 ---
    private Path applySwapMutation(Path path, int numSwaps) { /* ... 同前 ... */
        int n = this.problem.getLength(); if (n < 2) return path;
        int[] route = path.getCopyPath();
        for (int k = 0; k < numSwaps; k++) {
            int i = random.nextInt(n); int j; do { j = random.nextInt(n); } while (i == j);
            int temp = route[i]; route[i] = route[j]; route[j] = temp;
        } return new Path(route);
    }
    private void performSAIterationWithMixedNeighborhood() { /* ... 同前 ... */
        Path neighborPath; double delta; int[] currentRouteArray = this.currentPath.getCopyPath();
        if (random.nextDouble() < SWAP_PROBABILITY) {
            int n = this.problem.getLength(); if (n < 2) return; int i = random.nextInt(n); int j;
            do { j = random.nextInt(n); } while (i == j);
            delta = calculateSwapDelta(currentRouteArray, i, j);
            if (accept(delta)) { int temp = currentRouteArray[i]; currentRouteArray[i] = currentRouteArray[j]; currentRouteArray[j] = temp;
                acceptNeighbor(currentRouteArray, delta); }
        } else {
            int n = this.problem.getLength(); if (n < 4) return; int i, j;
            do { i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1));
            delta = calculate2OptDelta(currentRouteArray, i, j);
            if (accept(delta)) { reverseSegment(currentRouteArray, (i + 1) % n, j);
                acceptNeighbor(currentRouteArray, delta); }
        }
    }
    private double calculateSwapDelta(int[] route, int i, int j) { /* ... 同前 ... */
        int n = route.length; int city_i = route[i]; int city_j = route[j];
        int city_pi = route[(i - 1 + n) % n]; int city_ni = route[(i + 1) % n];
        int city_pj = route[(j - 1 + n) % n]; int city_nj = route[(j + 1) % n]; double delta = 0;
        if ((i + 1) % n == j) { delta = distances[city_pi][city_j] + distances[city_j][city_i] + distances[city_i][city_nj] - distances[city_pi][city_i] - distances[city_i][city_j] - distances[city_j][city_nj];
        } else if ((j + 1) % n == i) { delta = distances[city_pj][city_i] + distances[city_i][city_j] + distances[city_j][city_ni] - distances[city_pj][city_j] - distances[city_j][city_i] - distances[city_i][city_ni];
        } else { delta = distances[city_pi][city_j] + distances[city_j][city_ni] + distances[city_pj][city_i] + distances[city_i][city_nj] - distances[city_pi][city_i] - distances[city_i][city_ni] - distances[city_pj][city_j] - distances[city_j][city_nj];
        } return delta;
    }
    private double calculate2OptDelta(int[] r, int i, int j) { /* ... 同前 ... */
        int n = r.length; int a = r[i], b = r[(i + 1) % n], c = r[j], d = r[(j + 1) % n];
        return distances[a][c] + distances[b][d] - distances[a][b] - distances[c][d];
    }
    private boolean accept(double delta) { /* ... 同前 ... */
        return delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble();
    }
    private void acceptNeighbor(int[] newRouteArray, double delta) { /* ... 同前 ... */
        this.currentPath = new Path(newRouteArray); this.currentEvaluation += delta;
        if (this.currentEvaluation < this.bestEvaluation) { updateBestSolution(this.currentPath, this.currentEvaluation); this.bestPathIsLocallyOptimal = false; }
    }
    private void coolDown() { /* ... 同前 ... */
        this.currentTemperature *= COOLING_RATE; if (this.currentTemperature < EPSILON) this.currentTemperature = EPSILON;
    }
    private void updateBestSolution(Path path, double evaluation) { /* ... 同前 (含验证) ... */
        double costViaDelta = evaluation; double actualCost = this.evaluation.evaluate(path);
        if (Math.abs(costViaDelta - actualCost) > 1.0) {
            System.err.printf("!!! Cost Mismatch Detected !!! DeltaUpd: %.4f, Actual: %.4f%n", costViaDelta, actualCost);
            evaluation = actualCost; }
        if (evaluation < this.bestEvaluation - EPSILON) { this.bestPath = new Path(path); this.bestEvaluation = evaluation; System.out.printf("*** Global Best Updated: %.2f ***%n", this.bestEvaluation);
        } else if (Math.abs(evaluation - this.bestEvaluation) < EPSILON) { this.bestEvaluation = evaluation; }
        this.evaluation.evaluate(this.bestPath); // 确保框架用最新的 bestPath 或其 cost 更新
    }
    private void performIntensive2OptOnBestPath() { /* ... 同前 (First Improvement) ... */
        int n = this.problem.getLength(); if (n<4) {this.bestPathIsLocallyOptimal = true; return;}
        int[] route = this.bestPath.getCopyPath(); double currentBestEval = this.bestEvaluation;
        boolean improvedInThisStep = false;
        outer_loop:
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 2; j < n; j++) {
                if (i == 0 && j == n - 1) continue;
                double delta = calculate2OptDelta(route, i, j);
                if (delta < -EPSILON) {
                    reverseSegment(route, (i + 1) % n, j);
                    updateBestSolution(new Path(route), currentBestEval + delta);
                    improvedInThisStep = true;
                    // break outer_loop; // 找到第一个改进就结束本次调用，让下一次 loop 继续优化
                    return;
                } } }
        if (!improvedInThisStep) { this.bestPathIsLocallyOptimal = true; }
    }
    private void precomputeDistances(int n) { /* ... 同前 ... */
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) { Coordinates c1 = this.problem.getCoordinates(i);
            for (int j = i; j < n; j++) { if (i == j) this.distances[i][j] = 0;
            else { Coordinates c2 = this.problem.getCoordinates(j); double dist = c1.distance(c2);
                this.distances[i][j] = dist; this.distances[j][i] = dist; } } } }
    private Path createGreedyPath(int n) { /* ... 同前 ... */
        int[] route = new int[n]; boolean[] visited = new boolean[n]; int startNode = random.nextInt(n);
        route[0] = startNode; visited[startNode] = true; int currentCityIndex = startNode;
        for (int i = 1; i < n; i++) { int nearestNeighbor = -1; double minDistance = Double.POSITIVE_INFINITY;
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) { if (!visited[neighborIndex]) {
                double distance = this.distances[currentCityIndex][neighborIndex];
                if (distance < minDistance) { minDistance = distance; nearestNeighbor = neighborIndex; } } }
            if (nearestNeighbor == -1) { for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i)); }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true; currentCityIndex = nearestNeighbor;
        } return new Path(route); }
    private void reverseSegment(int[] route, int start, int end) { /* ... 同前 ... */
        int n = route.length; start = (start % n + n) % n; end = (end % n + n) % n; if (start == end) return;
        int p1 = start; int p2 = end; int segmentSize;
        if (p1 <= p2) { segmentSize = (p2 - p1 + 1) / 2; } else { segmentSize = (n - p1 + p2 + 1) / 2; }
        for (int k = 0; k < segmentSize; k++) { int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp;
            p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n; } }
    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) { /* ... 同前 ... */
        double avgDistance = 0; int count = 0;
        for (int i = 0; i < Math.min(n, 100); i++) { for (int j = i + 1; j < Math.min(n, 100); j++) {
            if (i < n && j < n) { avgDistance += distances[i][j]; count++; } } }
        if (count > 0) avgDistance /= count; else avgDistance = 100;
        double initialTemp = avgDistance * n * 0.1; return Math.max(1.0, initialTemp); }
}