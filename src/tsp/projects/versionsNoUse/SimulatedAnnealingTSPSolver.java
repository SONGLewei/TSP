package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 使用模拟退火 (Simulated Annealing) 算法解决TSP问题。
 * 基于 SimulatedAnnealingTSPSolver 的逻辑，适配当前框架。
 * 特点:
 * - 贪心算法初始化
 * - 2-opt 邻域生成
 * - 高效的 O(1) 成本增量 (delta) 计算
 * - Metropolis 接受准则
 * - 几何冷却策略
 * - 由外部框架控制时间
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @adaptation_source SimulatedAnnealingTSPSolver provided by user
 */
public class SimulatedAnnealingTSPSolver extends DemoProject { // Give it a new name

    // --- SA 参数 (这些值需要针对问题实例和时间限制进行仔细调优!) ---

    /**
     * 冷却率 (alpha)。值越接近1，冷却越慢。对于60秒，0.99999可能太慢了。
     * 尝试 0.995, 0.99, 0.985 等。
     */
    private static final double COOLING_RATE = 0.995; // 需要调优!

    /**
     * 最低温度。探索基本停止的阈值。
     */
    private static final double MIN_TEMPERATURE = 0.01; // 可以调优

    // --- 内部状态 ---
    // 使用 ThreadLocalRandom 可能在高并发下更好，但标准 Random 也可以
    // private ThreadLocalRandom random = ThreadLocalRandom.current();
    private Random random;
    private Path currentPath;       // 当前解 (Path object)
    private double currentEvaluation; // 当前解的评估值 (通过 delta 更新)
    private Path bestPath;          // 迄今为止找到的最佳解 (Path object)
    private double bestEvaluation;    // 最佳解的评估值 (从框架或 delta 计算验证)
    private double currentTemperature;  // 当前温度
    private double[][] distances;     // 预计算的距离矩阵，用于O(1) delta计算

    /**
     * 构造函数
     * @param evaluation 评估对象
     * @throws InvalidProjectException
     */
    public SimulatedAnnealingTSPSolver(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("你的名字");
        // this.addAuthor("你的队友名字"); // Add if applicable
        this.setMethodName("EfficientSA_2opt"); // 新名字
        this.random = new Random(); // Initialize standard Random
    }

    /**
     * 算法初始化。
     */
    @Override
    public void initialization() {
        int n = this.problem.getLength();

        // 1. 预计算距离矩阵 (关键优化!)
        precomputeDistances(n);

        // 2. 生成初始解 (使用贪心算法)
        // 注意: 贪心算法现在使用预计算距离
        this.currentPath = createGreedyPath(n);
        // 使用 evaluate 获取准确的初始成本并更新框架记录
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath);

        // 3. 初始化最佳解记录
        this.bestPath = new Path(this.currentPath); // 复制初始解
        this.bestEvaluation = this.currentEvaluation;

        // 4. 初始化温度 (使用新代码中的启发式，但基于框架的评估值)
        // 可以尝试不同的启发式，例如:
        // this.currentTemperature = calculateInitialTemperatureHeuristic1(this.currentEvaluation);
        this.currentTemperature = calculateInitialTemperatureHeuristic2(n); // 基于城市数量可能更稳定
        // this.currentTemperature = 1000.0; // 或者固定值开始调优

        System.out.println("EfficientSA Initialized. Greedy Path Cost: " + this.currentEvaluation);
        System.out.printf("Initial Temp: %.2f, Cooling Rate: %.5f%n", this.currentTemperature, COOLING_RATE);
    }

    /**
     * 算法主循环 - 执行单次 SA 迭代。
     */
    @Override
    public void loop() {
        // 如果温度过低，仅尝试局部改进（只接受更好的移动）
        if (this.currentTemperature <= MIN_TEMPERATURE) {
            performLocalImprovementOnly();
            // 仍然需要冷却，以防万一 MIN_TEMPERATURE 设置得过高
            coolDown();
            return;
        }

        int n = this.problem.getLength();
        // 获取当前路径数组的副本以进行修改和计算
        int[] currentRouteArray = this.currentPath.getCopyPath();

        // 1. 生成 2-opt 邻域的索引 i, j
        int i, j;
        // 循环直到找到有效的 i 和 j (确保 i < j 且不是相邻边)
        do {
            i = random.nextInt(n);
            j = random.nextInt(n);
            if (i == j) continue;
            if (i > j) { int temp = i; i = j; j = temp; } // 确保 i < j
        } while (j == i + 1 || (i == 0 && j == n - 1)); // 排除相邻边

        // 2. 高效计算成本增量 (Delta E) using O(1)
        int cityA_idx = currentRouteArray[i];
        int cityB_idx = currentRouteArray[(i + 1) % n]; // Handle wrap-around if i is last element conceptually
        int cityC_idx = currentRouteArray[j];
        int cityD_idx = currentRouteArray[(j + 1) % n]; // Handle wrap-around for j

        // 使用预计算的距离
        double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
        double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
        double delta = costAdded - costRemoved;

        // 3. Metropolis 接受准则
        if (delta < 0 || Math.exp(-delta / this.currentTemperature) > random.nextDouble()) {
            // 接受移动:
            // a) 在数组副本上执行 2-opt 翻转
            reverseSegment(currentRouteArray, i + 1, j);
            // b) 创建新的 Path 对象
            Path neighborPath = new Path(currentRouteArray);
            // c) 更新当前路径
            this.currentPath = neighborPath;
            // d) 更新当前评估值 (高效!)
            this.currentEvaluation += delta;

            // e) 检查是否是新的全局最优解
            if (this.currentEvaluation < this.bestEvaluation) {
                this.bestEvaluation = this.currentEvaluation;
                this.bestPath = this.currentPath; // 现在 currentPath 就是 bestPath
                // **重要**: 调用 evaluate 更新框架记录和图表，并可用于验证 delta 计算
                // double checkEval = this.evaluation.evaluate(this.bestPath);
                this.evaluation.evaluate(this.bestPath); // 必须调用以更新框架
                // System.out.printf("*** New Best Found: %.2f (Delta=%.2f, Temp=%.2f)%n",
                //                   this.bestEvaluation, delta, this.currentTemperature);
                // if (Math.abs(checkEval - this.bestEvaluation) > 1e-6) {
                //     System.err.println("Warning: Delta calculation mismatch!");
                // }
            }
        }
        // else: 不接受，currentPath 和 currentEvaluation 保持不变

        // 4. 冷却
        coolDown();
    }

    // --- SA 核心辅助方法 ---

    /** 降低温度 */
    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
    }

    /** 在低温时只执行接受更好解的局部改进 */
    private void performLocalImprovementOnly() {
        int n = this.problem.getLength();
        int[] currentRouteArray = this.currentPath.getCopyPath();

        int i, j;
        do {
            i = random.nextInt(n);
            j = random.nextInt(n);
            if (i == j) continue;
            if (i > j) { int temp = i; i = j; j = temp; }
        } while (j == i + 1 || (i == 0 && j == n - 1));

        int cityA_idx = currentRouteArray[i];
        int cityB_idx = currentRouteArray[(i + 1) % n];
        int cityC_idx = currentRouteArray[j];
        int cityD_idx = currentRouteArray[(j + 1) % n];

        double costRemoved = distances[cityA_idx][cityB_idx] + distances[cityC_idx][cityD_idx];
        double costAdded   = distances[cityA_idx][cityC_idx] + distances[cityB_idx][cityD_idx];
        double delta = costAdded - costRemoved;

        // 只接受严格改进的移动
        if (delta < 0) {
            reverseSegment(currentRouteArray, i + 1, j);
            Path neighborPath = new Path(currentRouteArray);
            this.currentPath = neighborPath;
            this.currentEvaluation += delta;

            // 检查是否也是全局最优
            if (this.currentEvaluation < this.bestEvaluation) {
                this.bestEvaluation = this.currentEvaluation;
                this.bestPath = this.currentPath;
                this.evaluation.evaluate(this.bestPath); // 更新框架
            }
        }
    }

    // --- 邻域和初始化辅助方法 ---

    /**
     * 预计算所有城市对之间的距离并存储。
     * @param n 城市数量
     */
    private void precomputeDistances(int n) {
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) {
            Coordinates c1 = this.problem.getCoordinates(i);
            // 计算到其他所有点的距离
            for (int j = i; j < n; j++) {
                if (i == j) {
                    this.distances[i][j] = 0;
                } else {
                    Coordinates c2 = this.problem.getCoordinates(j);
                    double dist = c1.distance(c2);
                    this.distances[i][j] = dist;
                    this.distances[j][i] = dist; // 对称TSP
                }
            }
        }
        System.out.println("Distance matrix precomputed.");
    }

    /**
     * 使用最近邻启发式算法（利用预计算距离）创建初始路径。
     * @param n 城市数量
     * @return 贪心生成的 Path 对象
     */
    private Path createGreedyPath(int n) {
        int[] route = new int[n];
        boolean[] visited = new boolean[n];
        int startNode = random.nextInt(n); // 随机起始点
        route[0] = startNode;
        visited[startNode] = true;
        int currentCityIndex = startNode;

        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1;
            double minDistance = Double.POSITIVE_INFINITY;

            // 使用预计算距离查找最近的未访问邻居
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) {
                if (!visited[neighborIndex]) {
                    // 直接从 distances 数组获取距离
                    double distance = this.distances[currentCityIndex][neighborIndex];
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestNeighbor = neighborIndex;
                    }
                }
            }

            // 健壮性检查
            if (nearestNeighbor == -1) {
                for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; }
                if (nearestNeighbor == -1) {
                    System.err.println("Error in greedy path: No unvisited city found.");
                    return new Path(Arrays.copyOf(route, i)); // Should not happen
                }
            }

            route[i] = nearestNeighbor;
            visited[nearestNeighbor] = true;
            currentCityIndex = nearestNeighbor;
        }
        // System.out.println("Greedy path created.");
        return new Path(route);
    }

    /**
     * 翻转路径数组中从索引 `start` 到 `end` (包含两者) 的部分。
     * @param route 路径数组
     * @param start 起始索引 (确保 start <= end)
     * @param end 结束索引
     */
    private void reverseSegment(int[] route, int start, int end) {
        // Modulo n is not needed here as i, j are calculated within [0, n-1]
        // and reverse is called with i+1, j where i < j.
        while (start < end) {
            int temp = route[start];
            route[start] = route[end];
            route[end] = temp;
            start++;
            end--;
        }
    }

    // --- 初始温度启发式计算 (示例) ---

    /**
     * 启发式 1: 基于初始成本计算初始温度 (来自 SimulatedAnnealingTSPSolver)
     * @param initialCost 初始路径成本
     * @return 初始温度估计值
     */
    private double calculateInitialTemperatureHeuristic1(double initialCost) {
        if (initialCost <= 0) return 1000.0; // Handle zero or negative cost case
        // 这个启发式可能需要调整因子 (e.g., / 5.0, / 20.0)
        return initialCost / 10.0;
    }

    /**
     * 启发式 2: 基于问题规模 (城市数量) 估算初始温度
     * (更简单，可能需要大幅调整)
     * @param numberOfCities 城市数量
     * @return 初始温度估计值
     */
    private double calculateInitialTemperatureHeuristic2(int numberOfCities) {
        // 非常粗略的估计，可能需要乘以一个与平均距离相关的因子
        // 这里的 10.0 只是一个示例因子，需要大量调优
        return (double)numberOfCities * 10.0;
    }

}