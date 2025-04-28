package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

// 必要时保留: import tsp.evaluation.Coordinates;
import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 使用模拟退火 (SA) + 后期 2-opt 局部搜索 + “大重启”机制解决TSP。
 *
 * "大重启" = 当最优解达到 2-opt 局部最优后，对其进行一次大幅度扰动
 * (反转长段落)，并以一个较低的温度重新开始 SA。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on SA_LS_Restart adding "Big Restart"
 */
public class SA_LS_BigRestart extends DemoProject { // 新名字

    // --- SA 参数 (需要调优!) ---
    private static final double COOLING_RATE = 0.995;          // 冷却率 (可调: 0.99, 0.998)
    private static final double MIN_TEMPERATURE_FACTOR = 0.01;     // 触发最终 LS/重启的温度因子 (可调: 0.005)
    // **** 主要修改点 ****
    private static final double RESTART_TEMPERATURE_FACTOR = 0.05; // 重启温度因子 (非常低! 可调: 0.01, 0.1)  还不错的
    //private static final double RESTART_TEMPERATURE_FACTOR = 0.1;
    // Perturbation strength 不再直接使用，改为下面的段落长度控制
    private static final double MIN_PERTURBATION_SEGMENT_RATIO = 0.25; // 扰动时反转段落的最小长度比例 (可调: 0.2, 0.3)（0.25 0.75）
    private static final double MAX_PERTURBATION_SEGMENT_RATIO = 0.75; // 扰动时反转段落的最大长度比例 (可调: 0.7, 0.8)


    // --- 内部状态 ---
    private Random random;
    private Path currentPath;
    private double currentEvaluation;
    private Path bestPath;           // 全局最优解
    private double bestEvaluation;
    private double currentTemperature;
    private double initialTemperature; // 存储初始温度用于重启
    private double minTemperature;     // 实际的最低温度阈值
    private double[][] distances;
    private boolean intensiveLocalSearchPhase = false;
    private boolean bestPathIs2OptOptimal = false;

    /** 构造函数 */
    public SA_LS_BigRestart(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("你的名字");
        // this.addAuthor("你的队友名字");
        this.setMethodName("SA_LS_BigRestart"); // 新名字
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

        this.initialTemperature = calculateInitialTemperatureHeuristic2(n);
        this.minTemperature = this.initialTemperature * MIN_TEMPERATURE_FACTOR;
        // 确保 minTemperature 不是极小值或零
        if (this.minTemperature < 1e-6) this.minTemperature = 1e-6;
        this.currentTemperature = this.initialTemperature;

        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false;

        System.out.println("SA_LS_BigRestart Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR);
    }

    /** 主循环 */
    @Override
    public void loop() {
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= this.minTemperature) {
            // System.out.println("--- Switching to Intensive 2-opt Local Search Phase ---");
            this.intensiveLocalSearchPhase = true;
            this.bestPathIs2OptOptimal = false;
        }

        if (this.intensiveLocalSearchPhase) {
            if (!this.bestPathIs2OptOptimal) {
                performIntensive2OptOnBestPath();
            } else {
                // System.out.println("--- Best path is 2-opt optimal. Initiating Big Restart ---");
                restartSearchWithLargePerturbation(); // 执行大重启
            }
        } else {
            performSAIteration();
            coolDown();
        }
    }

    // --- 重启逻辑 ---

    /**
     * 执行 "大重启": 对 bestPath 应用大幅度扰动 (反转长段落)，并重置状态。
     */
    private void restartSearchWithLargePerturbation() {
        // 1. 应用大幅度扰动
        Path perturbedPath = applyLargeSegmentReversalPerturbation(this.bestPath);

        // 2. 设置为当前路径并评估
        this.currentPath = perturbedPath;
        // 必须评估以获取扰动后的准确成本
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);

        // 3. 重置温度 (使用非常低的因子)
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR;
        // 确保重启温度不低于最低温度 (如果因子设置得极低)
        if (this.currentTemperature < this.minTemperature) {
            this.currentTemperature = this.minTemperature * 1.1; // 比min高一点点
            // 或者直接设为一个小的固定值
            // this.currentTemperature = 0.1;
        }
        if (this.currentTemperature <= 0) { // 避免0或负数
            this.currentTemperature = 1e-6;
        }


        // 4. 重置状态标志
        this.intensiveLocalSearchPhase = false; // 退出深度搜索，重新开始 SA
        this.bestPathIs2OptOptimal = false;

        // 5. 检查扰动后的路径是否意外成为新的全局最优
        // (这种情况很少见，但可能发生)
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }

        // System.out.printf("--- Big Restart completed. New current cost: %.2f, New temp: %.2f ---%n",
        //                  this.currentEvaluation, this.currentTemperature);
    }

    /**
     * 对路径应用一次大的扰动：随机选择并反转一个较长段落。
     * @param path 要扰动的路径
     * @return 扰动后的新 Path 对象
     */
    private Path applyLargeSegmentReversalPerturbation(Path path) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath();

        if (n < 4) { // 对太小的路径无法有效执行大扰动
            return path; // 返回原路径或执行简单扰动
        }

        int minLen = (int) Math.max(2, n * MIN_PERTURBATION_SEGMENT_RATIO);
        int maxLen = (int) Math.min(n - 2, n * MAX_PERTURBATION_SEGMENT_RATIO);
        if (maxLen <= minLen) { // 避免长度设置不合理
            minLen = 2;
            maxLen = n - 2;
        }

        int i = -1, j = -1;
        int attempts = 0;
        // 尝试几次找到符合长度要求的 i, j
        while(attempts < 100) { // 限制尝试次数避免死循环
            int idx1 = random.nextInt(n);
            int idx2 = random.nextInt(n);
            if (idx1 == idx2) { attempts++; continue; }
            if (idx1 > idx2) { int temp = idx1; idx1 = idx2; idx2 = temp; } // 确保 idx1 < idx2

            int segmentLength = idx2 - idx1;
            // 检查正向段长度
            if (segmentLength >= minLen && segmentLength <= maxLen) {
                i = idx1;
                j = idx2;
                break;
            }
            // 检查反向（环绕）段长度
            int wrapAroundLength = n - segmentLength;
            if (wrapAroundLength >= minLen && wrapAroundLength <= maxLen) {
                // 如果反向段符合，则反转反向段，等价于选择 j+1 到 i-1
                // 为了简化，我们还是反转 idx1+1 到 idx2，让 reverseSegment 处理
                // 但要确保选取的 i, j 确实定义了一个大段
                i = idx1;
                j = idx2;
                break; // 接受这对 i,j，反转它们之间的短段（效果等同反转长段）
                // 或者更精确地选择反转长段的端点:
                // i = idx2; j = idx1; // 如果 j<i 会在reverseSegment中处理，但逻辑需清晰
            }
            attempts++;
        }

        // 如果尝试多次找不到合适的 i,j (理论上不应发生)，执行一次随机 2-opt 作为保底
        if (i == -1 || j == -1 || (i == 0 && j == n - 1) || j == i+1) {
            // System.out.println("Warning: Could not find suitable large segment, performing random 2-opt instead.");
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1));
        }

        // System.out.printf("Applying large perturbation: reversing segment for indices [%d, %d]%n", i, j);
        // 注意：2-opt 反转的是 i+1 到 j
        reverseSegment(route, (i + 1) % n, j); // 使用 %n 保证索引有效性

        return new Path(route);
    }


    // --- SA 核心逻辑 (基本同前) ---
    private void performSAIteration() { /* ... 同 SA_LS_Restart ... */
        int n = this.problem.getLength();
        int[] currentRouteArray = this.currentPath.getCopyPath();
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
        double delta = costAdded - costRemoved;

        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            reverseSegment(currentRouteArray, (i + 1) % n, j); // 使用 %n
            Path neighborPath = new Path(currentRouteArray);
            this.currentPath = neighborPath;
            this.currentEvaluation += delta;
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
                this.bestPathIs2OptOptimal = false;
            }
        }
    }
    private void coolDown() { /* ... 同 SA_LS_Restart ... */
        this.currentTemperature *= COOLING_RATE;
        if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9; // 避免温度过低
    }
    private void updateBestSolution(Path path, double evaluation) { /* ... 同 SA_LS_Restart ... */
        if (evaluation < this.bestEvaluation - 1e-9) {
            this.bestPath = path;
            this.bestEvaluation = evaluation;
            this.evaluation.evaluate(this.bestPath);
            // System.out.printf("*** Global Best Updated: %.2f ***%n", this.bestEvaluation);
        } else if (Math.abs(evaluation - this.bestEvaluation) < 1e-9) {
            // 如果成本几乎一样，可能也更新 bestPath 引用，取决于策略
            // this.bestPath = path; // 可选
            this.bestEvaluation = evaluation; // 确保记录最新的（可能稍微好一点点）
            this.evaluation.evaluate(path); // 确保框架同步
        } else {
            // 这种情况理论上不应由 updateBestSolution 处理 (传入的 evaluation 更差)
            // 但为了健壮性，可以只同步框架
            this.evaluation.evaluate(this.bestPath); // 仍然用记录的 bestPath 更新框架
        }
    }

    // --- 2-opt 深度局部搜索逻辑 (基本同前) ---
    private void performIntensive2OptOnBestPath() { /* ... 同 SA_LS_Restart ... */
        int n = this.problem.getLength();
        int[] route = this.bestPath.getCopyPath();
        double currentBestEval = this.bestEvaluation;
        boolean improvedInThisStep = false;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 2; j < n; j++) {
                if (i == 0 && j == n - 1) continue;
                int cityA_idx = route[i];
                int cityB_idx = route[(i + 1) % n];
                int cityC_idx = route[j];
                int cityD_idx = route[(j + 1) % n];
                double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
                double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
                double delta = costAdded - costRemoved;

                if (delta < -1e-9) {
                    reverseSegment(route, (i + 1) % n, j); // 使用 %n
                    Path improvedPath = new Path(route);
                    double newEvaluation = currentBestEval + delta;
                    updateBestSolution(improvedPath, newEvaluation);
                    improvedInThisStep = true;
                    return; // 找到第一个改进就返回
                }
            }
        }

        if (!improvedInThisStep) {
            this.bestPathIs2OptOptimal = true; // 标记为已优化，准备重启
        }
    }


    // --- 共享的辅助方法 (基本同前) ---
    private void precomputeDistances(int n) { /* ... 同前 ... */
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) {
            Coordinates c1 = this.problem.getCoordinates(i);
            for (int j = i; j < n; j++) {
                if (i == j) this.distances[i][j] = 0;
                else {
                    Coordinates c2 = this.problem.getCoordinates(j);
                    double dist = c1.distance(c2);
                    this.distances[i][j] = dist;
                    this.distances[j][i] = dist;
                }
            }
        }
    }
    private Path createGreedyPath(int n) { /* ... 同前 ... */
        int[] route = new int[n];
        boolean[] visited = new boolean[n];
        int startNode = random.nextInt(n);
        route[0] = startNode; visited[startNode] = true;
        int currentCityIndex = startNode;
        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1;
            double minDistance = Double.POSITIVE_INFINITY;
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) {
                if (!visited[neighborIndex]) {
                    double distance = this.distances[currentCityIndex][neighborIndex];
                    if (distance < minDistance) {
                        minDistance = distance; nearestNeighbor = neighborIndex;
                    }
                }
            }
            if (nearestNeighbor == -1) {
                for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i));
            }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true;
            currentCityIndex = nearestNeighbor;
        }
        return new Path(route);
    }
    private void reverseSegment(int[] route, int start, int end) { /* ... 同前 ... */
        // 确保索引在 [0, n-1] 范围内，并且 start != end
        int n = route.length;
        start = start % n;
        end = end % n;

        // 标准的反转逻辑是反转 start 到 end (包含)
        // 如果 start > end，意味着这是一个环绕的反转
        if (start == end) return; // 无需反转

        // 简单的实现方式是假设我们总是反转较短的段
        // 更健壮（但可能复杂）的方式是处理环绕情况
        // 这里保持之前的简单逻辑：while (start < end)，适用于非环绕情况
        // 对于 2-opt 调用 (i+1, j) where i<j，这通常是正确的。
        // 对于扰动，如果随机选取的 i,j 导致 start > end，这里的简单实现可能行为不符合预期。
        // 一个更安全的 reverse 实现:
        int p1 = start;
        int p2 = end;
        int segmentSize; // 计算需要交换的对数

        if (p1 < p2) {
            segmentSize = (p2 - p1 + 1) / 2;
        } else { // 环绕情况
            segmentSize = (n - p1 + p2 + 1) / 2;
        }

        for (int k = 0; k < segmentSize; k++) {
            int temp = route[p1];
            route[p1] = route[p2];
            route[p2] = temp;
            p1 = (p1 + 1) % n;
            p2 = (p2 - 1 + n) % n; // +n 确保取模前是正数
        }
    }
    private double calculateInitialTemperatureHeuristic2(int numberOfCities) { /* ... 同前 ... */
        // 因子 10.0 需要调优
        return Math.max(1.0, (double)numberOfCities * 10.0); // 保证 T > 0
    }

}