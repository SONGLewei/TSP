package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates; // Keep if needed, but prefer distances array
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 使用模拟退火 (SA) 结合后期 2-opt 局部深度搜索来解决TSP问题。
 *
 * 过程:
 * 1. 正常执行模拟退火 (贪心初始解, 2-opt邻域, O(1) delta, Metropolis接受, 几何冷却)。
 * 2. 当温度低于 MIN_TEMPERATURE 时，切换到“深度局部搜索”阶段。
 * 3. 在深度局部搜索阶段，每次调用 loop() 会尝试对当前记录的 bestPath 执行一轮 2-opt 改进，
 * 直到 bestPath 对于 2-opt 达到局部最优。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on EfficientSA and user's deuxOp logic
 */
public class SA_with_2opt_LS extends DemoProject { // 新名字

    // --- SA 参数 (需要调优) ---
    private static final double COOLING_RATE = 0.995; // 示例值, 需要调优!
    private static final double MIN_TEMPERATURE = 0.1;  // 切换到深度局部搜索的阈值, 可以调优

    // --- 内部状态 ---
    private Random random;
    private Path currentPath;         // SA 当前解
    private double currentEvaluation;   // SA 当前解评估值 (通过 delta 更新)
    private Path bestPath;            // 全局最优解
    private double bestEvaluation;      // 全局最优解评估值
    private double currentTemperature;    // 当前温度
    private double[][] distances;       // 预计算距离矩阵
    private boolean intensiveLocalSearchPhase = false; // 是否进入深度局部搜索阶段
    private boolean bestPathIs2OptOptimal = false;     // 标记 bestPath 是否已达到 2-opt 局部最优

    /** 构造函数 */
    public SA_with_2opt_LS(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("你的名字");
        // this.addAuthor("你的队友名字");
        this.setMethodName("SA_with_2opt_LS"); // 新名字
        this.random = new Random();
    }

    /** 初始化 */
    @Override
    public void initialization() {
        int n = this.problem.getLength();
        precomputeDistances(n);
        this.currentPath = createGreedyPath(n); // 使用预计算距离的贪心
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);
        this.bestPath = new Path(this.currentPath);
        this.bestEvaluation = this.currentEvaluation;
        this.currentTemperature = calculateInitialTemperatureHeuristic2(n); // 选择或调整初始温度
        this.intensiveLocalSearchPhase = false; // 重置状态
        this.bestPathIs2OptOptimal = false;      // 重置状态

        System.out.println("SA_with_2opt_LS Initialized. Greedy Path Cost: " + this.currentEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f%n",
                this.currentTemperature, COOLING_RATE, MIN_TEMPERATURE);
    }

    /** 主循环 */
    @Override
    public void loop() {
        // 检查是否应该进入或停留在深度局部搜索阶段
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= MIN_TEMPERATURE) {
            System.out.println("--- Switching to Intensive 2-opt Local Search Phase ---");
            this.intensiveLocalSearchPhase = true;
            this.bestPathIs2OptOptimal = false; // 开始局部搜索前，假设 bestPath 未优化
        }

        if (this.intensiveLocalSearchPhase) {
            // 如果已经确定 bestPath 是 2-opt 最优，则 loop 不再做有效工作
            if (!this.bestPathIs2OptOptimal) {
                // 执行一轮 2-opt 深度优化
                performIntensive2OptOnBestPath();
            } else {
                // 可以选择在这里加一些微小的扰动再优化，或者直接返回等待时间耗尽
                // System.out.println("Best path is already 2-opt optimal. Waiting for time limit.");
            }
        } else {
            // --- 正常 SA 迭代 ---
            performSAIteration();
            // 冷却 (只在 SA 阶段进行)
            coolDown();
        }
    }

    // --- SA 核心逻辑 ---

    /** 执行一次标准的 SA 迭代 */
    private void performSAIteration() {
        int n = this.problem.getLength();
        int[] currentRouteArray = this.currentPath.getCopyPath();

        // 1. 生成 2-opt 邻域索引 i, j
        int i, j;
        do {
            i = random.nextInt(n); j = random.nextInt(n);
            if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
        } while (j == i + 1 || (i == 0 && j == n - 1));

        // 2. 高效计算 Delta E
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
            reverseSegment(currentRouteArray, i + 1, j);
            Path neighborPath = new Path(currentRouteArray);
            this.currentPath = neighborPath;
            this.currentEvaluation += delta; // 高效更新

            // 检查是否是新的全局最优
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
                // 找到了更好的解，重置 2-opt 优化状态，因为新解可能不是 2-opt 最优
                this.bestPathIs2OptOptimal = false;
            }
        }
        // else: 不接受
    }

    /** 降低温度 */
    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
    }

    /** 更新全局最优解 */
    private void updateBestSolution(Path path, double evaluation) {
        this.bestPath = path; // 直接引用，因为 SA 接受时会创建新 Path
        this.bestEvaluation = evaluation;
        // **重要**: 调用 evaluate 更新框架记录和图表
        this.evaluation.evaluate(this.bestPath);
        // System.out.printf("*** SA found New Best: %.2f (Temp=%.2f)%n", this.bestEvaluation, this.currentTemperature);
    }

    // --- 2-opt 深度局部搜索逻辑 ---

    /**
     * 对当前的 bestPath 执行一轮迭代式 2-opt 优化。
     * 类似于你的 deuxOp.loop() 中的逻辑，但只执行一轮寻找和应用改进。
     * 如果找到改进，更新 bestPath 和 bestEvaluation，并返回。
     * 如果一整轮都没有找到改进，则将 bestPathIs2OptOptimal 标记为 true。
     */
    private void performIntensive2OptOnBestPath() {
        // System.out.println("Performing intensive 2-opt step on best path...");
        int n = this.problem.getLength();
        // **重要**: 对 bestPath 的副本进行操作，找到改进后再更新 bestPath
        int[] route = this.bestPath.getCopyPath();
        double currentBestEval = this.bestEvaluation; // 当前最优值
        boolean improvedInThisStep = false;

        // 迭代式 2-opt 逻辑 (类似于你的 deuxOp.loop 内的 for 循环)
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 2; j < n; j++) {
                // 排除相邻边 (包括首尾相连)
                if (i == 0 && j == n - 1) continue;

                // 计算 delta (使用 O(1) 方法)
                int cityA_idx = route[i];
                int cityB_idx = route[(i + 1) % n];
                int cityC_idx = route[j];
                int cityD_idx = route[(j + 1) % n];
                double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
                double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
                double delta = costAdded - costRemoved;

                // 如果找到改进 (delta < 0)
                // 为了避免浮点误差，可以用一个小的 epsilon: delta < -1e-9
                if (delta < -1e-9) {
                    // 应用 2-opt 翻转到 route 数组上
                    reverseSegment(route, i + 1, j);

                    // 创建新的 Path 对象
                    Path improvedPath = new Path(route);
                    // 计算新的评估值 (基于 delta)
                    double newEvaluation = currentBestEval + delta;

                    // 更新全局最优解
                    updateBestSolution(improvedPath, newEvaluation);
                    // System.out.printf("--- Intensive 2-opt improved best: %.2f (Delta=%.2f)%n", newEvaluation, delta);

                    improvedInThisStep = true;
                    // 找到第一个改进后就立即返回，让下一次 loop 调用继续优化
                    // 这模拟了你的 break 逻辑，并适应了框架的 loop 结构
                    return;
                }
            }
        }

        // 如果完整遍历了一遍都没有找到改进
        if (!improvedInThisStep) {
            // System.out.println("--- Best path is now 2-opt optimal. ---");
            this.bestPathIs2OptOptimal = true; // 标记为已优化
        }
    }


    // --- 共享的辅助方法 ---

    /** 预计算距离 (同前) */
    private void precomputeDistances(int n) {
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
        // System.out.println("Distance matrix precomputed.");
    }

    /** 贪心初始解 (同前) */
    private Path createGreedyPath(int n) {
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
            if (nearestNeighbor == -1) { // Fallback
                for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i));
            }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true;
            currentCityIndex = nearestNeighbor;
        }
        return new Path(route);
    }

    /** 翻转数组片段 (同前) */
    private void reverseSegment(int[] route, int start, int end) {
        while (start < end) {
            int temp = route[start];
            route[start] = route[end];
            route[end] = temp;
            start++; end--;
        }
    }

    /** 计算初始温度的启发式方法 (同前) */
    private double calculateInitialTemperatureHeuristic2(int numberOfCities) {
        // 这个因子 10.0 需要根据你的问题实例调整
        return (double)numberOfCities * 10.0;
    }
}