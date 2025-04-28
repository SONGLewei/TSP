package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.CompetitorProject;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 结合了 SA, 2-opt, 以及 Mutating Hill Climbing (MHC) / Iterated Local Search (ILS) 思想的混合算法。
 *
 * 过程:
 * 1. 主体使用 SA 进行搜索，主要邻域操作为 2-opt。
 * 2. 当温度降低，切换到对当前最优解进行 2-opt 深度局部搜索。
 * 3. 当 2-opt 局部搜索完成（达到局部最优）后，对该最优解执行多次 Swap 操作（模拟 MHC 的 Mutation）。
 * 4. 以较低温度从被 Swap 扰动后的解重新开始 SA。
 * 5. 重复直到时间耗尽。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on SA_LS_BigRestart incorporating MHC/ILS ideas (Swap Mutation)
 */
public class SA_MHC_Hybrid extends DemoProject { // 新名字

    // --- 参数 (需要调优!) ---
    private static final double COOLING_RATE = 0.998;          // 冷却率 (可以尝试稍慢)
    private static final double MIN_TEMPERATURE_FACTOR = 0.005;    // 触发最终 LS/重启的温度因子
    private static final double RESTART_TEMPERATURE_FACTOR = 0.05; // 重启温度因子 (保持较低)
    // **** 修改点: 扰动参数 ****
    private static final int MUTATION_SWAPS = 50; // 重启/变异时执行的 Swap 次数 (代替之前的扰动强度, 可调: 20-100)

    // --- 内部状态 ---
    private Random random;
    private Path currentPath;
    private double currentEvaluation;
    private Path bestPath;           // 全局最优解
    private double bestEvaluation;
    private double currentTemperature;
    private double initialTemperature;
    private double minTemperature;
    private double[][] distances;
    private boolean intensiveLocalSearchPhase = false;
    private boolean bestPathIs2OptOptimal = false;

    /** 构造函数 */
    public SA_MHC_Hybrid(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("你的名字");
        // this.addAuthor("你的队友名字");
        this.setMethodName("SA_MHC_Hybrid"); // 新名字
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
        if (this.minTemperature < 1e-6) this.minTemperature = 1e-6;
        this.currentTemperature = this.initialTemperature;

        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false;

        System.out.println("SA_MHC_Hybrid Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f, Mutation Swaps: %d%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR, MUTATION_SWAPS);
    }

    /** 主循环 */
    @Override
    public void loop() {
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= this.minTemperature) {
            this.intensiveLocalSearchPhase = true;
            this.bestPathIs2OptOptimal = false;
        }

        if (this.intensiveLocalSearchPhase) {
            if (!this.bestPathIs2OptOptimal) {
                performIntensive2OptOnBestPath(); // 使用 2-opt 进行深度局部搜索
            } else {
                restartSearchWithSwapMutation(); // 使用 Swap 进行扰动/变异并重启
            }
        } else {
            performSAIteration(); // 主要使用 2-opt 进行 SA 探索
            coolDown();
        }
    }

    // --- 重启/变异逻辑 ---

    /**
     * 执行重启/变异：对 bestPath 应用多次 Swap 操作，并重置状态。
     */
    private void restartSearchWithSwapMutation() {
        // 1. 应用 Swap 变异
        Path mutatedPath = applySwapMutation(this.bestPath, MUTATION_SWAPS);

        // 2. 设置为当前路径并评估
        this.currentPath = mutatedPath;
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath); // 需准确评估

        // 3. 重置温度
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
        if (this.currentTemperature < this.minTemperature) this.currentTemperature = this.minTemperature * 1.1;
        if (this.currentTemperature <= 0) this.currentTemperature = 1e-6;

        // 4. 重置状态标志
        this.intensiveLocalSearchPhase = false; // 退出深度搜索，重新开始 SA
        this.bestPathIs2OptOptimal = false;

        // 5. 检查变异后的路径是否意外成为新的全局最优
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }
        // System.out.printf("--- Restarted with Swap Mutation. New current cost: %.2f, New temp: %.2f ---%n",
        //                   this.currentEvaluation, this.currentTemperature);
    }

    /**
     * 对给定的路径应用指定次数的随机 Swap 操作作为变异/扰动。
     * @param path 要变异的路径
     * @param numSwaps 执行 Swap 的次数
     * @return 变异后的新 Path 对象
     */
    private Path applySwapMutation(Path path, int numSwaps) {
        int n = this.problem.getLength();
        if (n < 2) return path; // Swap 至少需要 2 个城市

        int[] route = path.getCopyPath(); // 在副本上操作

        for (int k = 0; k < numSwaps; k++) {
            // 随机选择两个不同的索引
            int i = random.nextInt(n);
            int j;
            do {
                j = random.nextInt(n);
            } while (i == j);

            // 执行交换
            int temp = route[i];
            route[i] = route[j];
            route[j] = temp;
        }
        return new Path(route);
    }


    // --- SA 核心逻辑 (主要使用 2-opt) ---

    /** 执行一次 SA 迭代 (主要使用 2-opt) */
    private void performSAIteration() {
        int n = this.problem.getLength();
        if (n < 4) return; // 2-opt 需要足够城市

        int[] currentRouteArray = this.currentPath.getCopyPath();

        // 1. 生成 2-opt 邻域索引 i, j
        int i, j;
        do {
            i = random.nextInt(n); j = random.nextInt(n);
            if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
        } while (j == i + 1 || (i == 0 && j == n - 1));

        // 2. 高效计算 2-opt Delta E
        int cityA_idx = currentRouteArray[i];
        int cityB_idx = currentRouteArray[(i + 1) % n];
        int cityC_idx = currentRouteArray[j];
        int cityD_idx = currentRouteArray[(j + 1) % n];
        double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
        double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
        double delta = costAdded - costRemoved;

        // 3. Metropolis 接受准则
        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            // 接受移动
            reverseSegment(currentRouteArray, (i + 1) % n, j);
            Path neighborPath = new Path(currentRouteArray);
            this.currentPath = neighborPath;
            this.currentEvaluation += delta; // 高效更新

            // 检查是否是新的全局最优
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
                this.bestPathIs2OptOptimal = false; // 新最优解需要重新 2-opt 优化
            }
        }
        // else: 不接受
    }


    private void coolDown() { /* ... 同前 ... */
        this.currentTemperature *= COOLING_RATE;
        if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9;
    }
    private void updateBestSolution(Path path, double evaluation) { /* ... 同前 ... */
        if (evaluation < this.bestEvaluation - 1e-9) {
            this.bestPath = path; this.bestEvaluation = evaluation; this.evaluation.evaluate(this.bestPath);
        } else if (Math.abs(evaluation - this.bestEvaluation) < 1e-9) {
            this.bestEvaluation = evaluation; this.evaluation.evaluate(path);
        } else { this.evaluation.evaluate(this.bestPath); }
    }

    // --- 2-opt 深度局部搜索逻辑 (保持不变) ---
    private void performIntensive2OptOnBestPath() { /* ... 同前 ... */
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

    // --- 共享的辅助方法 ---
    private void precomputeDistances(int n) { /* ... 同前 ... */
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
    private Path createGreedyPath(int n) { /* ... 同前 ... */
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
    private void reverseSegment(int[] route, int start, int end) { /* ... 同前 (使用改进版) ... */
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
    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) { /* ... 同前 ... */
        double avgDistance = 0; int count = 0;
        for (int i = 0; i < Math.min(n, 100); i++) {
            for (int j = i + 1; j < Math.min(n, 100); j++) {
                if (i < n && j < n) { avgDistance += distances[i][j]; count++; }
            } }
        if (count > 0) avgDistance /= count; else avgDistance = 100;
        double initialTemp = avgDistance * n * 0.1; // 示例因子 0.1
        return Math.max(1.0, initialTemp);
    }
    // 保留备选
    private double calculateInitialTemperatureHeuristic2(int numberOfCities) {
        return Math.max(1.0, (double)numberOfCities * 10.0);
    }
}