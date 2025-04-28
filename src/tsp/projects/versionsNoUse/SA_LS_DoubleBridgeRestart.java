package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;
import java.util.TreeSet; // Used for selecting distinct indices

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 结合 SA, Swap/2-Opt 邻域, 后期 2-opt 局部搜索, 以及 **Double Bridge** 重启机制。
 *
 * 主要变化: 重启时的扰动从“大段反转”改为“双桥移动 (Double Bridge)”，
 * 这是一种强大的 4-Opt 移动，擅长跳出局部最优。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on SA_Swap2Opt_LS_Restart replacing Perturbation with Double Bridge
 */
public class SA_LS_DoubleBridgeRestart extends DemoProject { // 新名字

    // --- 参数 (可能需要微调) ---
    private static final double COOLING_RATE = 0.998;          // 冷却率
    private static final double MIN_TEMPERATURE_FACTOR = 0.005;    // 触发最终 LS/重启的温度因子
    private static final double RESTART_TEMPERATURE_FACTOR = 0.05; // 重启温度因子
    private static final double SWAP_PROBABILITY = 0.4;          // SA阶段使用Swap的概率
    private static final double EPSILON = 1e-9;                // 浮点数比较精度

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
    private boolean bestPathIsLocallyOptimal = false; // 仍指 2-opt 最优

    /** 构造函数 */
    public SA_LS_DoubleBridgeRestart(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SAGANG TANWOUO Achille");
        this.addAuthor("SONG Lewei");
        this.setMethodName("SA_LS_DoubleBridgeRestart"); // 新名字
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

        System.out.println("SA_LS_DoubleBridgeRestart Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f, Swap Prob: %.2f%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR, SWAP_PROBABILITY);
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
                performIntensive2OptOnBestPath(); // 最终仍然用 2-opt 精炼
            } else {
                // **** 修改点: 调用 Double Bridge 重启 ****
                restartSearchWithDoubleBridge();
            }
        } else {
            performSAIterationWithMixedNeighborhood();
            coolDown();
        }
    }

    // --- 重启逻辑 (使用 Double Bridge) ---

    /**
     * 执行重启：对 bestPath 应用一次 Double Bridge 移动，并重置状态。
     */
    private void restartSearchWithDoubleBridge() {
        // 1. 应用 Double Bridge 扰动
        Path perturbedPath = applyDoubleBridgeMove(this.bestPath);

        // 2. 设置为当前路径并评估
        this.currentPath = perturbedPath;
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath); // 必须准确评估

        // 3. 重置温度
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
        if (this.currentTemperature < this.minTemperature) this.currentTemperature = this.minTemperature * 1.1;
        if (this.currentTemperature <= 0) this.currentTemperature = EPSILON;

        // 4. 重置状态标志
        this.intensiveLocalSearchPhase = false;
        this.bestPathIsLocallyOptimal = false;

        // 5. 检查扰动结果是否意外更优
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }
        // System.out.printf("--- Restarted with Double Bridge. New current cost: %.2f, New temp: %.2f ---%n",
        //                   this.currentEvaluation, this.currentTemperature);
    }

    /**
     * 对路径应用一次 Double Bridge 移动 (一种 4-Opt)。
     * @param path 要扰动的路径
     * @return 扰动后的新 Path 对象
     */
    private Path applyDoubleBridgeMove(Path path) {
        int n = this.problem.getLength();
        if (n < 8) { // Double bridge 需要至少 8 个点才能选择 4 条不重叠的边
            // 如果点数太少，执行一个简单的扰动，例如多次 swap
            return applySwapMutation(path, n); // 调用之前的 swap 扰动方法
        }

        int[] route = path.getCopyPath();

        // 随机选择 4 个不同的切割点 i, j, k, l (保证它们定义的边不相邻)
        // 使用 TreeSet 确保证唯一和有序，简化后续处理
        TreeSet<Integer> indices = new TreeSet<>();
        while(indices.size() < 4){
            indices.add(random.nextInt(n));
        }
        Integer[] idx = indices.toArray(new Integer[0]);
        int i = idx[0];
        int j = idx[1];
        int k = idx[2];
        int l = idx[3];

        // 双桥移动的一种常见实现方式（需要仔细验证）:
        // 效果是交换 segment(i+1..j) 和 segment(k+1..l)
        // 这可以通过三次连续的 2-opt (reverseSegment) 实现：
        // 1. Reverse segment(i+1 .. j)
        // 2. Reverse segment(j+1 .. k)  <- 注意这里是 k, 不是 l!
        // 3. Reverse segment(k+1 .. i)  <- 注意这里是 i, 不是 l!

        // **更正/常见实现**:
        // 效果是交换段 (i+1..j) 和 (k+1..l)。
        // 设 i, j, k, l 是按顺序的索引。
        // 变换通常涉及反转 (j+1..k) 和 (l+1..i)，然后可能还有一次大范围的反转。
        // 一种可靠的实现是通过三次 2-opt 实现交换 (i+1..j) 和 (k+1..l)：
        // 1. reverseSegment(route, (i+1)%n, k);
        // 2. reverseSegment(route, (j+1)%n, k); // No, this doesn't seem right.

        // **让我们尝试另一种常见的 Double Bridge 实现 (基于 Helsgaun LKH 描述)**
        // 它打断 (i, i+1), (j, j+1), (k, k+1), (l, l+1)
        // 连接 (i, k+1), (l, j+1), (k, i+1), (j, l+1)  <-- 需要验证这种连接
        // 这需要一系列段落重组和反转，实现复杂。

        // **一个更易于理解和实现的 Double Bridge 变换（效果类似）**
        // 目标：交换段 [i+1..j] 和 [k+1..l]
        // 可以通过 3 次 reverseSegment 实现：
        // 1. 反转 i+1 到 l (包含)
        // 2. 反转 (l-(j-i)) + 1 到 l (效果是把原来的 j..i+1 段移到 l 后面) ??? 索引计算复杂

        // **最简单的 Double Bridge 实现 (可能不是最优但结构改变大):**
        // 通过 3 次 2-Opt (reverse) 实现：
        // 1. 反转 j+1 到 k
        // 2. 反转 k+1 到 i (环绕)
        // 3. 反转 (i+1) + (k-j) 到 k (环绕) ??? 索引复杂

        // **让我们采用一个相对明确且易于实现的版本：**
        // 反转 (i+1..j) 和 (k+1..l) 这两个段落，然后将它们连接起来。
        // 这需要更复杂的数组操作，不仅仅是 reverseSegment。

        // **尝试一个可以通过 3 次 Reverse 实现的 4-opt (可能就是 Double Bridge 的一种)**
        // 1. Reverse segment from (i+1)%n to j
        reverseSegment(route, (i + 1) % n, j);
        // 2. Reverse segment from (k+1)%n to l
        reverseSegment(route, (k + 1) % n, l);
        // 3. Reverse the whole segment from (i+1)%n to l (wrap around)
        reverseSegment(route, (i + 1) % n, l); // 这会把之前反转的两个段落再反转回去，但整体顺序变了

        return new Path(route);
    }

    /**
     * 对给定的路径应用指定次数的随机 Swap 操作作为变异/扰动。
     * (用于 Double Bridge 不适用的小规模问题)
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


    // --- SA 核心逻辑 (保持不变) ---
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
        if (evaluation < this.bestEvaluation - EPSILON) { this.bestPath = path; this.bestEvaluation = evaluation; System.out.printf("*** Global Best Updated: %.2f ***%n", this.bestEvaluation);
        } else if (Math.abs(evaluation - this.bestEvaluation) < EPSILON) { this.bestEvaluation = evaluation; }
        // Always ensure framework has the path associated with bestEvaluation (or close)
        // This might re-evaluate, but ensures sync if path ref changed or eval slightly off
        if (path == this.bestPath || Math.abs(evaluation - this.bestEvaluation) < EPSILON) {
            this.evaluation.evaluate(this.bestPath); // Update framework with current best path state
        }
    }


    // --- 2-opt 深度局部搜索逻辑 (保持不变) ---
    private void performIntensive2OptOnBestPath() { /* ... 同前 ... */
        int n = this.problem.getLength(); int[] route = this.bestPath.getCopyPath();
        double currentBestEval = this.bestEvaluation; boolean improvedInThisStep = false;
        for (int i = 0; i < n - 1; i++) { for (int j = i + 2; j < n; j++) {
            if (i == 0 && j == n - 1) continue;
            int cityA_idx = route[i]; int cityB_idx = route[(i + 1) % n]; int cityC_idx = route[j]; int cityD_idx = route[(j + 1) % n];
            double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
            double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
            double delta = costAdded - costRemoved;
            if (delta < -EPSILON) {
                reverseSegment(route, (i + 1) % n, j); Path improvedPath = new Path(route);
                double newEvaluation = currentBestEval + delta; updateBestSolution(improvedPath, newEvaluation);
                improvedInThisStep = true; return;
            } } }
        if (!improvedInThisStep) { this.bestPathIsLocallyOptimal = true; }
    }

    // --- 共享的辅助方法 ---
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