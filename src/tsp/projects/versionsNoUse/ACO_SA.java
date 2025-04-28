package tsp.projects.versionsNoUse; // Or your specific package

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.CompetitorProject;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

import javax.swing.*;

/**
 * 结合蚁群优化 (ACO) 和模拟退火 (SA) 的混合算法。
 *
 * 流程:
 * 1. 初始化信息素矩阵。
 * 2. 在每次迭代 (loop 调用) 中:
 * a. 多只蚂蚁根据信息素和距离启发式构建路径。
 * b. 对每只蚂蚁构建的路径进行一次短暂的 SA 局部优化。
 * c. 更新全局最优路径。
 * d. 更新信息素矩阵 (蒸发 + 蚂蚁沉积)。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on ACO + SA description
 */
public class ACO_SA extends DemoProject { // 新名字

    // --- ACO 参数 (需要调优!) ---
    private static final int    NUM_ANTS = 25;       // 蚂蚁数量 (可调: e.g., 10, n/5, n/2)
    private static final double ALPHA = 1.0;       // 信息素影响因子 (可调: e.g., 0.5 - 2.0)
    private static final double BETA = 3;        // 启发式信息(距离倒数)影响因子 (可调: e.g., 1.0 - 5.0)
    private static final double RHO = 0.15;         // 信息素蒸发率 (0 < rho < 1, 可调: e.g., 0.05 - 0.5)
    private static final double Q = 1.0;       // 信息素沉积强度常数 (可调, 与路径成本尺度有关)
    private static final double INITIAL_PHEROMONE = 1.0; // 初始信息素值 (可调)
    // ACO 变种相关 (例如 AS, EAS, ACS, MMAS), 这里使用基础 Ant System (AS) 的更新方式

    // --- SA 局部搜索参数 (用于蚂蚁路径优化, 需要快速!) ---
    private static final int    SA_LOCAL_ITERATIONS = 50; // 每只蚂蚁路径的 SA 优化迭代次数 (非常短! 可调)
    private static final double SA_LOCAL_COOLING_RATE = 0.85; // SA 局部优化的快速冷却率 (可调: 0.8 - 0.95)
    // SA 局部优化的初始温度可以在运行时基于路径成本动态设定

    // --- 内部状态 ---
    private Random random;
    private Path bestPath;            // 全局最优解
    private double bestEvaluation;      // 全局最优评估值
    private double[][] distances;       // 预计算距离
    private double[][] pheromones;      // 信息素矩阵
    private double[][] heuristicInfo;   // 启发式信息矩阵 (1/distance)^beta

    private static final double EPSILON = 1e-9;

    /** 构造函数 */
    public ACO_SA(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SAGANG TANWOUO Achille");
        this.addAuthor("SONG Lewei");
        this.setMethodName("ACO_SA_Hybrid"); // 新名字
        this.random = new Random();
    }

    /** 初始化 */
    @Override
    public void initialization() {
        int n = this.problem.getLength();
        precomputeDistances(n);
        initializePheromones(n);
        calculateHeuristicInfo(n); // 预计算启发式信息

        // 初始化一个最优路径 (可以用贪心, 或第一个蚂蚁的结果)
        this.bestPath = createGreedyPath(n); // 或者随机?
        this.bestEvaluation = this.evaluation.evaluate(this.bestPath);

        System.out.println("ACO_SA_Hybrid Initialized. Initial Best Path Cost: " + this.bestEvaluation);
        // 打印 ACO 和 SA 参数...
    }

    /** 主循环: 执行一代 ACO + SA 局部搜索 */
    @Override
    public void loop() {
        int n = this.problem.getLength();
        if (n <= 1) return; // 无法执行

        // 存储本代蚂蚁构建的路径和评估值
        List<Path> antPaths = new ArrayList<>(NUM_ANTS);
        List<Double> antEvals = new ArrayList<>(NUM_ANTS);
        double[][] pheromoneDelta = new double[n][n]; // 存储本代信息素增量

        // 1. 所有蚂蚁构建路径 + SA 局部优化
        for (int ant = 0; ant < NUM_ANTS; ant++) {
            // a. 构建路径
            Path rawPath = buildAntTour(n);
            // b. SA 局部优化
            Path optimizedPath = runSALocalSearch(rawPath, SA_LOCAL_ITERATIONS);
            // c. 评估优化后的路径
            double optimizedEval = this.evaluation.evaluate(optimizedPath); // 必须评估

            antPaths.add(optimizedPath);
            antEvals.add(optimizedEval);

            // d. 更新全局最优解
            if (optimizedEval < this.bestEvaluation) {
                // 注意: updateBestSolution 内部也调用了 evaluate
                updateBestSolution(optimizedPath, optimizedEval);
                // 可以考虑在这里进行精英蚂蚁策略: 给全局最优路径额外增加信息素
            }

            // e. 计算该蚂蚁贡献的信息素增量 (Ant System 方式)
            calculatePheromoneDelta(pheromoneDelta, optimizedPath, optimizedEval);
        }

        // 2. 更新信息素矩阵
        updatePheromones(n, pheromoneDelta);

        // (可选) 可以在这里加入其他 ACO 策略，如最大最小蚂蚁系统 (MMAS) 的信息素限制
    }

    // --- ACO 核心方法 ---

    /** 初始化信息素矩阵 */
    private void initializePheromones(int n) {
        this.pheromones = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                // 避免初始化为 0，通常设为一个小的正值
                this.pheromones[i][j] = INITIAL_PHEROMONE;
            }
        }
    }

    /** 预计算启发式信息 (1/distance)^beta */
    private void calculateHeuristicInfo(int n){
        this.heuristicInfo = new double[n][n];
        for(int i=0; i<n; ++i){
            for(int j=0; j<n; ++j){
                if(i == j){
                    this.heuristicInfo[i][j] = 0; // 不可能去同一个城市
                } else {
                    // 防止除以零或极小距离导致数值问题
                    double distance = Math.max(distances[i][j], EPSILON);
                    this.heuristicInfo[i][j] = Math.pow(1.0 / distance, BETA);
                }
            }
        }
    }


    /** 一只蚂蚁构建一条路径 */
    private Path buildAntTour(int n) {
        int[] tour = new int[n];
        boolean[] visited = new boolean[n];
        int startNode = random.nextInt(n); // 随机起点
        tour[0] = startNode;
        visited[startNode] = true;
        int currentCity = startNode;

        for (int step = 1; step < n; step++) {
            int nextCity = selectNextCity(currentCity, visited, n);
            tour[step] = nextCity;
            visited[nextCity] = true;
            currentCity = nextCity;
        }
        return new Path(tour);
    }

    /** 根据信息素和启发式信息，概率性选择下一个城市 */
    private int selectNextCity(int currentCity, boolean[] visited, int n) {
        double[] probabilities = new double[n];
        double probabilitiesSum = 0.0;

        // 计算所有未访问邻居的选择概率
        for (int next = 0; next < n; next++) {
            if (!visited[next]) {
                // 确保信息素不为 0 或负数 (根据 ACO 变种可能需要处理)
                double pheromoneLevel = Math.max(pheromones[currentCity][next], EPSILON);
                double heuristicValue = heuristicInfo[currentCity][next]; // 已预计算

                probabilities[next] = Math.pow(pheromoneLevel, ALPHA) * heuristicValue; // heuristicValue 已包含 beta 次方
                probabilitiesSum += probabilities[next];
            } else {
                probabilities[next] = 0.0; // 不能访问已访问过的城市
            }
        }

        // 轮盘赌选择
        int selectedCity = -1;
        if (probabilitiesSum <= 0) {
            // 如果所有邻居概率和为0（可能发生在早期或所有邻居都访问过），随机选一个未访问的
            List<Integer> unvisited = new ArrayList<>();
            for(int i=0; i<n; ++i) if(!visited[i]) unvisited.add(i);
            if(!unvisited.isEmpty()){
                selectedCity = unvisited.get(random.nextInt(unvisited.size()));
            } else {
                // 理论上不应发生，除非 n=step
                System.err.println("Error: No unvisited city to select!");
                // 随便返回一个，或者抛异常
                for(int i=0; i<n; ++i) if(i != currentCity) return i;
                return (currentCity + 1) % n; // 最后的保护
            }
        } else {
            double randDouble = random.nextDouble() * probabilitiesSum;
            double cumulativeProb = 0.0;
            for (int next = 0; next < n; next++) {
                if (!visited[next]) {
                    cumulativeProb += probabilities[next];
                    if (cumulativeProb >= randDouble) {
                        selectedCity = next;
                        break;
                    }
                }
            }
            // 如果因为浮点误差没有选到，选最后一个概率不为0的
            if (selectedCity == -1) {
                for(int i=n-1; i>=0; --i) if(!visited[i] && probabilities[i] > 0) {selectedCity = i; break;}
                // 再次检查，如果还没选到（概率极低），随机选
                if(selectedCity == -1) {
                    List<Integer> unvisited = new ArrayList<>();
                    for(int i=0; i<n; ++i) if(!visited[i]) unvisited.add(i);
                    if(!unvisited.isEmpty()){
                        selectedCity = unvisited.get(random.nextInt(unvisited.size()));
                    } else { // Fallback
                        for(int i=0; i<n; ++i) if(i != currentCity) return i;
                        return (currentCity + 1) % n;
                    }
                }
            }
        }
        return selectedCity;
    }

    /** 更新信息素矩阵 */
    private void updatePheromones(int n, double[][] pheromoneDelta) {
        // 1. 蒸发
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                pheromones[i][j] *= (1.0 - RHO);
                // 可以设置信息素下限 (Min-Max Ant System)
                // pheromones[i][j] = Math.max(pheromones[i][j], MIN_PHEROMONE_LIMIT);
            }
        }
        // 2. 沉积增量
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                pheromones[i][j] += pheromoneDelta[i][j];
                // 可以设置信息素上限 (Min-Max Ant System)
                // pheromones[i][j] = Math.min(pheromones[i][j], MAX_PHEROMONE_LIMIT);
            }
        }
    }

    /** 计算单只蚂蚁贡献的信息素增量 */
    private void calculatePheromoneDelta(double[][] deltaMatrix, Path path, double eval) {
        int n = path.getPath().length;
        int[] route = path.getPath();
        // Ant System: Q / L (L 是路径长度)
        double depositAmount = Q / Math.max(eval, EPSILON); // 防止除以零

        for (int i = 0; i < n; i++) {
            int city1 = route[i];
            int city2 = route[(i + 1) % n]; // 连接回起点
            // 在对称 TSP 中，双向增加
            deltaMatrix[city1][city2] += depositAmount;
            deltaMatrix[city2][city1] += depositAmount;
        }
    }


    // --- SA 局部搜索方法 ---
    /**
     * 对给定的路径运行一个快速的 SA 局部搜索进行优化。
     * @param initialPath 蚂蚁构建的原始路径
     * @param maxIterations SA 迭代次数
     * @return 经过 SA 优化后的路径
     */
    private Path runSALocalSearch(Path initialPath, int maxIterations) {
        int n = initialPath.getPath().length;
        if (n < 2) return initialPath; // 太短无法优化

        Path currentSAPath = new Path(initialPath); // 从蚂蚁路径开始
        double currentSAEval = this.evaluation.quickEvaluate(currentSAPath); // 快速评估初始成本

        // 动态设置初始温度 (基于当前路径成本的启发式)
        double initialSATemp = calculateSALocalInitialTemp(currentSAEval);
        double currentSATemp = initialSATemp;
        double finalSATemp = initialSATemp * 1e-3; // 设定一个快速冷却的目标末温
        double saAlpha = Math.pow(finalSATemp / initialSATemp, 1.0 / maxIterations); // 计算快速冷却率
        if (saAlpha >= 1.0) saAlpha = SA_LOCAL_COOLING_RATE; // 使用固定快速冷却率作为备用

        for (int iter = 0; iter < maxIterations; iter++) {
            if (currentSATemp <= finalSATemp) break; // 温度过低则停止

            // 使用 2-Opt 作为邻域操作 (也可以混合 Swap)
            int[] currentRouteArray = currentSAPath.getCopyPath();
            Path neighborSAPath = null;
            double delta = Double.POSITIVE_INFINITY;

            if (n >= 4) {
                int i, j;
                do { i = random.nextInt(n); j = random.nextInt(n); if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
                } while (j == i + 1 || (i == 0 && j == n - 1));
                delta = calculate2OptDelta(currentRouteArray, i, j);

                // 不在这里应用变换，只计算 delta
            } else { continue; } // 点太少无法 2-Opt


            // SA 接受准则
            if (delta < 0 || Math.exp(-delta / currentSATemp) > random.nextDouble()) {
                // 接受，现在应用变换
                int i=-1, j=-1; // 需要重新获取或传递 i,j - 这是之前代码结构的问题
                // 为简化，我们假设 delta 是有效的，并应用一个随机 2-opt (不理想)
                // 或者，更健壮的是在计算 delta 时就生成邻居路径
                // 这里我们生成邻居路径
                int[] saRouteCopy = currentSAPath.getCopyPath(); // 用当前路径副本
                do { i = random.nextInt(n); j = random.nextInt(n); if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
                } while (j == i + 1 || (i == 0 && j == n - 1));
                delta = calculate2OptDelta(saRouteCopy, i, j); // 重新计算 delta 确保一致

                if (delta < 0 || Math.exp(-delta / currentSATemp) > random.nextDouble()) { // 再次检查接受性
                    reverseSegment(saRouteCopy, (i + 1) % n, j);
                    currentSAPath = new Path(saRouteCopy);
                    currentSAEval += delta; // 用 O(1) 更新成本
                }
            }
            // 冷却
            currentSATemp *= saAlpha;
        }
        return currentSAPath; // 返回优化后的路径
    }

    /** 计算 SA 局部搜索的初始温度 */
    private double calculateSALocalInitialTemp(double pathCost) {
        // 可以用一个简单的比例，或者更复杂的基于 delta 估计的方法
        // 目标是让初始几步有较高的接受率
        return Math.max(1.0, pathCost * 0.05); // 示例：成本的 5% 作为初始温度 (需要调优!)
    }


    // --- 共享的辅助方法 ---
    private void precomputeDistances(int n) { /* ... 同前 ... */
        this.distances = new double[n][n]; for (int i = 0; i < n; i++) { Coordinates c1 = this.problem.getCoordinates(i); for (int j = i; j < n; j++) { if (i == j) this.distances[i][j] = 0; else { Coordinates c2 = this.problem.getCoordinates(j); double dist = c1.distance(c2); this.distances[i][j] = dist; this.distances[j][i] = dist; } } } }
    private Path createGreedyPath(int n) { /* ... 同前 ... */
        int[] route = new int[n]; boolean[] visited = new boolean[n]; int startNode = random.nextInt(n); route[0] = startNode; visited[startNode] = true; int currentCityIndex = startNode;
        for (int i = 1; i < n; i++) { int nearestNeighbor = -1; double minDistance = Double.POSITIVE_INFINITY; for (int neighborIndex = 0; neighborIndex < n; neighborIndex++) { if (!visited[neighborIndex]) { double distance = this.distances[currentCityIndex][neighborIndex]; if (distance < minDistance) { minDistance = distance; nearestNeighbor = neighborIndex; } } }
            if (nearestNeighbor == -1) { for(int k=0; k<n; ++k) if (!visited[k]) { nearestNeighbor = k; break; } if (nearestNeighbor == -1) return new Path(Arrays.copyOf(route, i)); }
            route[i] = nearestNeighbor; visited[nearestNeighbor] = true; currentCityIndex = nearestNeighbor; } return new Path(route); }
    private void reverseSegment(int[] route, int start, int end) { /* ... 同前 ... */
        int n = route.length; start = (start % n + n) % n; end = (end % n + n) % n; if (start == end) return;
        int p1 = start; int p2 = end; int segmentSize; if (p1 <= p2) { segmentSize = (p2 - p1 + 1) / 2; } else { segmentSize = (n - p1 + p2 + 1) / 2; }
        for (int k = 0; k < segmentSize; k++) { int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp; p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n; } }
    private double calculate2OptDelta(int[] r, int i, int j) { /* ... 同前 ... */
        int n = r.length; int a = r[i], b = r[(i + 1) % n], c = r[j], d = r[(j + 1) % n]; return distances[a][c] + distances[b][d] - distances[a][b] - distances[c][d]; }
    private void updateBestSolution(Path path, double evaluation) { /* ... 同前 ... */
        double costViaDelta = evaluation; double actualCost = this.evaluation.evaluate(path); if (Math.abs(costViaDelta - actualCost) > 1.0) { /*System.err.printf("!!! Cost Mismatch ...%n");*/ evaluation = actualCost; }
        if (evaluation < this.bestEvaluation - EPSILON) { this.bestPath = new Path(path); this.bestEvaluation = evaluation; System.out.printf("*** Global Best Updated (ACO+SA): %.2f ***%n", this.bestEvaluation); }
        else if (Math.abs(evaluation - this.bestEvaluation) < EPSILON) { this.bestEvaluation = evaluation; } this.evaluation.evaluate(this.bestPath); }
    private double calculateInitialTemperatureHeuristic(int n, double[][] distances) { /* ... 同前 ... */
        double avgDistance = 0; int count = 0; for (int i = 0; i < Math.min(n, 100); i++) { for (int j = i + 1; j < Math.min(n, 100); j++) { if (i < n && j < n) { avgDistance += distances[i][j]; count++; } } }
        if (count > 0) avgDistance /= count; else if (n > 1) avgDistance = distances[0][1]; else avgDistance = 100;
        double initialTemp = avgDistance * n * 1; return Math.max(1.0, initialTemp); } // Factor 1.0

}