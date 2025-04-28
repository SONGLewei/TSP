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
 * 使用模拟退火 (SA) 结合后期 2-opt 局部深度搜索，并加入重启机制来解决TSP问题。
 *
 * 过程:
 * 1. 正常 SA 运行。
 * 2. 低温时切换到 2-opt 深度局部搜索优化 bestPath。
 * 3. 当 bestPath 达到 2-opt 最优后，进行一次强力扰动并重启 SA (重置温度)。
 * 4. 重复此过程直到时间耗尽。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on SA_with_2opt_LS adding Restart
 */
public class SA_LS_Restart extends DemoProject { // 新名字

    // --- SA 参数 (需要调优) ---
    private static final double COOLING_RATE = 0.998;      // 冷却率
    private static final double MIN_TEMPERATURE_FACTOR = 0.01; // 用于计算实际最低温度的因子 (乘以初始温度)
    private static final double RESTART_TEMPERATURE_FACTOR = 0.1; // 重启时温度设为初始温度的多少倍
    private static final int PERTURBATION_STRENGTH = 25; // 重启时扰动的强度 (执行多少次随机 2-opt)

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
    public SA_LS_Restart(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("你的名字");
        // this.addAuthor("你的队友名字");
        this.setMethodName("SA_LS_Restart"); // 新名字
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

        // 计算和存储初始及最低温度
        this.initialTemperature = calculateInitialTemperatureHeuristic2(n); // 或者其他启发式
        this.minTemperature = this.initialTemperature * MIN_TEMPERATURE_FACTOR;
        this.currentTemperature = this.initialTemperature;

        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false;

        System.out.println("SA_LS_Restart Initialized. Greedy Path Cost: " + this.bestEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f, Min Temp for LS: %.2f, Restart Temp Factor: %.2f%n",
                this.initialTemperature, COOLING_RATE, this.minTemperature, RESTART_TEMPERATURE_FACTOR);
    }

    /** 主循环 */
    @Override
    public void loop() {
        // 检查是否应进入深度局部搜索
        if (!this.intensiveLocalSearchPhase && this.currentTemperature <= this.minTemperature) {
            // System.out.println("--- Switching to Intensive 2-opt Local Search Phase ---");
            this.intensiveLocalSearchPhase = true;
            this.bestPathIs2OptOptimal = false; // 准备优化 bestPath
        }

        if (this.intensiveLocalSearchPhase) {
            if (!this.bestPathIs2OptOptimal) {
                // 执行一轮 2-opt 深度优化
                performIntensive2OptOnBestPath(); // 这个方法内部会在优化完成后设置 bestPathIs2OptOptimal = true
            } else {
                // bestPath 已经是 2-opt 最优，执行重启
                // System.out.println("--- Best path is 2-opt optimal. Initiating Restart ---");
                restartSearch();
            }
        } else {
            // --- 正常 SA 迭代 ---
            performSAIteration();
            // 冷却 (只在 SA 阶段进行)
            coolDown();
        }
    }

    // --- 重启逻辑 ---

    /**
     * 对当前 bestPath 进行扰动，并重置温度等状态，以启动新一轮 SA 搜索。
     */
    private void restartSearch() {
        // 1. 扰动 bestPath
        Path perturbedPath = applyPerturbation(this.bestPath, PERTURBATION_STRENGTH);

        // 2. 设置为当前路径
        this.currentPath = perturbedPath;
        this.currentEvaluation = this.evaluation.quickEvaluate(this.currentPath); // 重新评估扰动后的路径

        // 3. 重置温度
        this.currentTemperature = this.initialTemperature * RESTART_TEMPERATURE_FACTOR; // 重置为初始温度的一部分

        // 4. 重置状态标志
        this.intensiveLocalSearchPhase = false;
        this.bestPathIs2OptOptimal = false; // 新的搜索阶段开始

        // 5. （可选）检查扰动后的路径是否意外地成为新的最优解
        if (this.currentEvaluation < this.bestEvaluation) {
            updateBestSolution(this.currentPath, this.currentEvaluation);
        }

        // System.out.printf("--- Restarted search. New current cost: %.2f, New temp: %.2f ---%n",
        //                  this.currentEvaluation, this.currentTemperature);
    }

    /**
     * 对给定的路径应用指定次数的随机 2-opt 移动作为扰动。
     * @param path 要扰动的路径
     * @param strength 扰动强度（执行多少次 2-opt）
     * @return 扰动后的新 Path 对象
     */
    private Path applyPerturbation(Path path, int strength) {
        int n = this.problem.getLength();
        int[] route = path.getCopyPath(); // 在副本上操作

        for (int k = 0; k < strength; k++) {
            // 执行一次随机的 2-opt 移动 (不需要检查是否改进)
            int i, j;
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1)); // 避免无效移动

            reverseSegment(route, i + 1, j);
        }
        return new Path(route);
    }


    // --- SA 核心逻辑 (与 SA_with_2opt_LS 基本相同) ---

    private void performSAIteration() {
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
            reverseSegment(currentRouteArray, i + 1, j);
            Path neighborPath = new Path(currentRouteArray);
            this.currentPath = neighborPath;
            this.currentEvaluation += delta;
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
                this.bestPathIs2OptOptimal = false; // 新最优解需要重新局部优化
            }
        }
    }

    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
        // 确保温度不会低得离谱 (可选)
        // if (this.currentTemperature < 1e-9) this.currentTemperature = 1e-9;
    }

    private void updateBestSolution(Path path, double evaluation) {
        // 确保评估值确实降低了（考虑浮点误差）
        if (evaluation < this.bestEvaluation - 1e-9) {
            this.bestPath = path;
            this.bestEvaluation = evaluation;
            this.evaluation.evaluate(this.bestPath); // 通知框架
            // System.out.printf("*** Global Best Updated: %.2f ***%n", this.bestEvaluation);
        } else {
            // 如果只是浮点误差导致的微小“改进”，可能不需要更新 bestPath 引用
            // 但仍然需要更新 bestEvaluation 和通知框架
            this.bestEvaluation = evaluation;
            this.evaluation.evaluate(path); // 用传入的 path 更新，即使它可能与 this.bestPath 引用相同
        }
    }


    // --- 2-opt 深度局部搜索逻辑 (与 SA_with_2opt_LS 相同) ---

    private void performIntensive2OptOnBestPath() {
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

                if (delta < -1e-9) { // 考虑浮点误差
                    reverseSegment(route, i + 1, j);
                    Path improvedPath = new Path(route);
                    double newEvaluation = currentBestEval + delta;
                    updateBestSolution(improvedPath, newEvaluation); // 使用辅助方法更新
                    // System.out.printf("--- Intensive 2-opt improved best: %.2f (Delta=%.2f)%n", newEvaluation, delta);
                    improvedInThisStep = true;
                    // 找到第一个改进就返回，让下一次 loop 调用继续
                    return;
                }
            }
        }

        if (!improvedInThisStep) {
            // System.out.println("--- Best path is now 2-opt optimal (before potential restart). ---");
            this.bestPathIs2OptOptimal = true; // 标记为已优化，准备重启
        }
    }


    // --- 共享的辅助方法 (与 EfficientSA 相同) ---

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
        while (start < end) {
            int temp = route[start];
            route[start] = route[end];
            route[end] = temp;
            start++; end--;
        }
    }
    private double calculateInitialTemperatureHeuristic2(int numberOfCities) { /* ... 同前 ... */
        // 因子 10.0 仍然需要调优
        return (double)numberOfCities * 10.0;
    }

}