package tsp.projects.versionsNoUse; // 或者 tsp.projects.yourPackage

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 使用模拟退火 (Simulated Annealing) 算法解决TSP问题。
 * 结合了贪心初始化和2-opt邻域搜索，并通过Metropolis准则概率性接受较差解以跳出局部最优。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on_analysis Gemini AI
 */
public class SimulatedAnnealingTSP extends DemoProject { // 确保继承正确的基类

    // --- SA 参数 (这些值需要针对问题实例和时间限制进行仔细调优!) ---

    /**
     * 初始温度。需要足够高以允许早期广泛探索。
     * 其值应与路径成本变化的典型幅度相关。
     * 示例值，需要调优。
     */
    private static final double INITIAL_TEMPERATURE = 1000.0;

    /**
     * 冷却率 (alpha)。值越接近1，冷却越慢，探索越充分，但需要更多时间。
     * 值越小，冷却越快，可能过早收敛。对于60秒限制，可能需要比0.9995更快的速率。
     * 示例值，需要调优。
     */
    private static final double COOLING_RATE = 0.995; // 尝试 0.99, 0.995, 0.985 等

    /**
     * 最低温度。当温度低于此值时，接受差解的概率极低，探索基本停止。
     * 示例值，需要调优。
     */
    private static final double MIN_TEMPERATURE = 0.1;

    // --- 内部状态 ---
    private Random random;
    private Path currentPath;       // 当前解
    private double currentEvaluation; // 当前解的评估值
    private Path bestPath;          // 迄今为止找到的最佳解
    private double bestEvaluation;    // 最佳解的评估值
    private double currentTemperature;  // 当前温度
    private double[][] distances;     // 预计算的距离矩阵，用于加速评估

    /**
     * 构造函数
     * @param evaluation 评估对象
     * @throws InvalidProjectException
     */
    public SimulatedAnnealingTSP(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        // 设置作者和方法名
        this.addAuthor("你的名字");
        if (/* 你有队友 */ true) {
            this.addAuthor("你的队友名字");
        }
        this.setMethodName("SimulatedAnnealing_2opt"); // 给你的算法起个名字
    }

    /**
     * 算法初始化。在主循环开始前执行一次。
     */
    @Override
    public void initialization() {
        this.random = new Random();
        int n = this.problem.getLength();

        // 1. 预计算距离矩阵 (关键优化!)
        precomputeDistances(n);

        // 2. 生成初始解 (使用贪心算法)
        this.currentPath = createGreedyPath(n);
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath); // 使用 evaluate 获取初始成本并更新框架

        // 3. 初始化最佳解记录
        this.bestPath = new Path(this.currentPath); // 复制初始解
        this.bestEvaluation = this.currentEvaluation;

        // 4. 初始化温度
        this.currentTemperature = INITIAL_TEMPERATURE;

        // 打印初始信息（可选）
        System.out.println("SA Initialized. Greedy Path Cost: " + this.currentEvaluation);
        System.out.println("Initial Temp: " + this.currentTemperature + ", Cooling Rate: " + COOLING_RATE);
    }

    /**
     * 算法主循环。会被外部调用器反复执行，直到时间耗尽。
     * 每次调用执行一步或几步SA迭代。这里设计为执行一步。
     */
    @Override
    public void loop() {
        // 如果温度过低，探索意义不大，但仍需让框架控制时间
        if (this.currentTemperature <= MIN_TEMPERATURE) {
            // 可以选择在这里做一些非常局部的优化，或者简单返回
            // 保持运行以消耗时间，但不再进行概率接受
            performLocalImprovementIfNeeded(); // 例如，只接受更好的邻居
            return; // 或者继续执行下面的冷却步骤
        }

        // 1. 生成邻域解 (使用 2-opt)
        Path neighborPath = generateNeighbor(this.currentPath);

        // 2. 评估邻域解 (使用快速评估)
        // 注意：这里使用 quickEvaluate，它不会更新全局最佳记录，速度可能更快
        // 但如果 quickEvaluate 内部实现仍然是 O(n) 完整计算，则优化有限
        double neighborEvaluation = this.evaluation.quickEvaluate(neighborPath);

        // 3. 计算成本变化
        double deltaE = neighborEvaluation - this.currentEvaluation;

        // 4. Metropolis 接受准则
        if (deltaE < 0) {
            // 如果邻域解更好，总是接受
            acceptNeighbor(neighborPath, neighborEvaluation);

            // 检查是否是全局最优解
            if (neighborEvaluation < this.bestEvaluation) {
                updateBestSolution(neighborPath, neighborEvaluation);
            }
        } else {
            // 如果邻域解更差，根据概率接受
            double acceptanceProbability = Math.exp(-deltaE / this.currentTemperature);
            if (random.nextDouble() < acceptanceProbability) {
                acceptNeighbor(neighborPath, neighborEvaluation);
            }
            // else: 不接受，保留 currentPath 和 currentEvaluation
        }

        // 5. 冷却
        coolDown();

        // 可以在这里添加一些日志记录当前状态（可选）
        // System.out.printf("Temp: %.2f, CurrentEval: %.2f, BestEval: %.2f%n",
        //                   currentTemperature, currentEvaluation, bestEvaluation);
    }

    // --- SA 核心辅助方法 ---

    /**
     * 接受邻域解为当前解。
     * @param neighborPath 被接受的邻域路径
     * @param neighborEvaluation 被接受的邻域路径的评估值
     */
    private void acceptNeighbor(Path neighborPath, double neighborEvaluation) {
        this.currentPath = neighborPath;
        this.currentEvaluation = neighborEvaluation;
    }

    /**
     * 更新全局最优解记录。
     * @param newBestPath 新的最优路径
     * @param newBestEvaluation 新的最优评估值
     */
    private void updateBestSolution(Path newBestPath, double newBestEvaluation) {
        this.bestPath = new Path(newBestPath); // 创建副本以防后续修改 currentPath
        this.bestEvaluation = newBestEvaluation;
        // **重要**: 调用 evaluate 更新框架记录和可能的图表
        this.evaluation.evaluate(this.bestPath);
        // System.out.println("*** New Best Found: " + this.bestEvaluation + " at Temp: " + this.currentTemperature);
    }

    /**
     * 降低温度。
     */
    private void coolDown() {
        this.currentTemperature *= COOLING_RATE;
    }

    /**
     * 在低温时可能执行的局部改进（例如，只接受更好的移动）。
     * 这是一个可选的增强。
     */
    private void performLocalImprovementIfNeeded() {
        Path neighborPath = generateNeighbor(this.currentPath);
        double neighborEvaluation = this.evaluation.quickEvaluate(neighborPath);
        if (neighborEvaluation < this.currentEvaluation) {
            acceptNeighbor(neighborPath, neighborEvaluation);
            if (neighborEvaluation < this.bestEvaluation) {
                updateBestSolution(neighborPath, neighborEvaluation);
            }
        }
    }


    // --- 邻域生成 (2-opt) ---

    /**
     * 生成当前路径的一个 2-opt 邻域路径。
     * @param current 当前路径
     * @return 一个新的邻域路径
     */
    private Path generateNeighbor(Path current) {
        int n = this.problem.getLength();
        int[] currentRoute = current.getCopyPath(); // 获取副本进行修改

        int i, j;
        // 循环直到找到有效的 i 和 j (确保 i < j 且不是相邻边)
        do {
            i = random.nextInt(n);
            j = random.nextInt(n);
            // 确保 i 和 j 不同，并且如果 j < i，则交换它们
            if (i == j) continue;
            if (i > j) { int temp = i; i = j; j = temp; }
            // 排除选择相邻边的情况 (0 和 n-1 是相邻的)
        } while (j == i + 1 || (i == 0 && j == n - 1));

        // 执行 2-opt 翻转 (翻转从 i+1 到 j 的部分)
        reverseSegment(currentRoute, i + 1, j);

        return new Path(currentRoute);
    }

    /**
     * 翻转路径数组中从索引 `from` 到 `to` (包含两者) 的部分。
     * @param route 路径数组
     * @param from 起始索引
     * @param to 结束索引
     */
    private void reverseSegment(int[] route, int from, int to) {
        int n = route.length;
        // 处理索引，因为它们可能来自随机选择，需要规范化
        from = from % n;
        to = to % n;

        // 确保 from <= to (逻辑上)
        // 如果不是，则翻转的是 "环绕" 的部分，更复杂，这里简化处理标准情况
        // 在 generateNeighbor 中已确保 i < j 且非相邻，所以这里的 from <= to
        // (注意: 2-opt 的原始定义是移除 (i, i+1) 和 (j, j+1), 连接 (i, j) 和 (i+1, j+1)，
        //  等价于翻转 i+1 到 j 的段)

        while (from < to) {
            int temp = route[from];
            route[from] = route[to];
            route[to] = temp;
            from++;
            to--;
        }
    }


    // --- 初始化辅助方法 ---

    /**
     * 预计算所有城市对之间的距离并存储。
     * @param n 城市数量
     */
    private void precomputeDistances(int n) {
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) {
            Coordinates c1 = this.problem.getCoordinates(i);
            for (int j = i; j < n; j++) { // 对称距离
                if (i == j) {
                    this.distances[i][j] = 0;
                } else {
                    Coordinates c2 = this.problem.getCoordinates(j);
                    double dist = c1.distance(c2);
                    this.distances[i][j] = dist;
                    this.distances[j][i] = dist; // 利用对称性
                }
            }
        }
    }

    /**
     * 使用最近邻启发式算法创建初始路径。
     * @param n 城市数量
     * @return 贪心生成的路径
     */
    private Path createGreedyPath(int n) {
        int[] route = new int[n];
        boolean[] visited = new boolean[n];
        // 随机选择起始城市
        int startNode = random.nextInt(n);
        route[0] = startNode;
        visited[startNode] = true;
        int currentCityIndex = startNode;

        for (int i = 1; i < n; i++) {
            int nearestNeighbor = -1;
            double minDistance = Double.POSITIVE_INFINITY;

            // 使用预计算的距离查找最近的未访问邻居
            for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) {
                if (!visited[neighborIndex]) {
                    double distance = this.distances[currentCityIndex][neighborIndex];
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestNeighbor = neighborIndex;
                    }
                }
            }

            // 处理所有城市都已访问的情况（理论上不应在循环内发生）
            if (nearestNeighbor == -1) {
                // 寻找任何未访问的城市作为备用（健壮性）
                for(int k=0; k<n; ++k) {
                    if (!visited[k]) {
                        nearestNeighbor = k;
                        break;
                    }
                }
                // 如果仍然找不到，说明逻辑有误或问题特殊
                if (nearestNeighbor == -1 && i < n) {
                    System.err.println("Error in greedy path construction: Cannot find next unvisited city.");
                    // 可以选择抛出异常或返回部分路径
                    return new Path(Arrays.copyOf(route, i));
                }
            }

            route[i] = nearestNeighbor;
            visited[nearestNeighbor] = true;
            currentCityIndex = nearestNeighbor;
        }

        return new Path(route);
    }
}