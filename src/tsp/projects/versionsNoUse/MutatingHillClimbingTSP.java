package tsp.projects.versionsNoUse; // Or your specific package

import java.util.Arrays;
import java.util.Random;

import tsp.evaluation.Coordinates;
import tsp.evaluation.Evaluation;
import tsp.evaluation.Path;
import tsp.projects.DemoProject;
import tsp.projects.InvalidProjectException;

/**
 * 严格按照 Mutating Hill Climbing (MHC) 流程实现的算法。
 * (实质上是一种 Iterated Local Search)
 *
 * 流程:
 * 1. 随机初始化路径。
 * 2. 执行近似 "Best Improvement" Hill Climbing (通过随机采样邻居):
 * - 在当前路径附近随机采样 N 个 2-opt 邻居。
 * - 如果找到改进邻居，移动到其中最好的一个。
 * - 重复此过程。
 * 3. 如果采样邻居中没有更好的，则认为到达局部最优。
 * 4. 更新全局最优解记录。
 * 5. 对全局最优解执行多次 Swap 变异。
 * 6. 将变异后的解设为当前解，返回第 2 步。
 *
 * @author [你的名字] 和 [你的队友名字, 如果有]
 * @based_on Mutating Hill Climbing description provided by user
 */
public class MutatingHillClimbingTSP extends DemoProject { // 新名字

    // --- 参数 (需要调优!) ---
    // 在 Hill Climbing 阶段，每次 loop 调用检查多少个随机 2-Opt 邻居
    // 可以设为 n, 2n, n*log(n) 等，需要实验
    private static final int NEIGHBOR_SAMPLE_MULTIPLIER = 2; // e.g., check 2*n neighbors per loop
    // 变异/扰动强度：执行多少次 Swap
    private static final int MUTATION_SWAPS = 50;           // 可调: 20-100

    // --- 内部状态 ---
    private Random random;
    private Path currentPath;         // 当前正在优化的路径
    private double currentEvaluation;   // 当前路径的评估值
    private Path bestPath;            // 全局找到的最优路径
    private double bestEvaluation;      // 全局最优评估值
    private double[][] distances;
    private boolean needsMutation = false; // 状态：当前是否需要执行变异?

    private static final double EPSILON = 1e-9; // 浮点数比较精度


    /** 构造函数 */
    public MutatingHillClimbingTSP(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        this.addAuthor("SAGANG TANWOUO Achille");
        this.addAuthor("SONG Lewei");
        this.setMethodName("MutatingHillClimbing"); // 新名字
        this.random = new Random();
    }

    /** 初始化 */
    @Override
    public void initialization() {
        int n = this.problem.getLength();
        precomputeDistances(n);

        // **** 按照 MHC 图示：随机初始化 ****
        this.currentPath = new Path(Path.getRandomPath(n));
        this.currentEvaluation = this.evaluation.evaluate(this.currentPath); // 评估初始解

        // 初始化全局最优
        this.bestPath = new Path(this.currentPath);
        this.bestEvaluation = this.currentEvaluation;

        this.needsMutation = false; // 初始状态是进行爬山

        System.out.println("MutatingHillClimbing Initialized. Random Path Cost: " + this.bestEvaluation);
        System.out.printf("Neighbor Sample Size per Loop (approx): %d * n, Mutation Swaps: %d%n",
                NEIGHBOR_SAMPLE_MULTIPLIER, MUTATION_SWAPS);
    }

    /** 主循环：执行一步 MHC 流程 */
    @Override
    public void loop() {
        if (needsMutation) {
            // --- 执行变异阶段 ---
            // System.out.println("Mutating best path...");
            // 1. 对全局最优解 bestPath 进行变异
            Path mutatedPath = applySwapMutation(this.bestPath, MUTATION_SWAPS);
            // 2. 将变异后的解作为新的当前解
            this.currentPath = mutatedPath;
            this.currentEvaluation = this.evaluation.evaluate(this.currentPath); // 评估新起点
            // 3. 重置状态，准备下一轮爬山
            this.needsMutation = false;
            // 4. （可选）检查变异结果是否意外更好
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
            }
        } else {
            // --- 执行爬山阶段 (寻找最佳改进邻居 - 抽样近似) ---
            findAndMoveToBestNeighbor();
        }
    }

    /**
     * 查找当前路径的邻居（通过抽样），并移动到其中最好的一个改进邻居。
     * 如果抽样中没有找到改进邻居，则设置 needsMutation = true。
     */
    private void findAndMoveToBestNeighbor() {
        int n = this.problem.getLength();
        if (n < 4) { // 2-opt 需要至少 4 个点
            this.needsMutation = true; // 无法爬山，直接准备变异
            return;
        }

        Path bestNeighborFound = null; // 本次抽样中找到的最佳邻居
        double bestNeighborEval = this.currentEvaluation; // 最佳邻居的评估值
        double bestDelta = 0; // 最佳邻居带来的改进量 (负数)

        int sampleSize = n * NEIGHBOR_SAMPLE_MULTIPLIER; // 计算本次循环要采样的邻居数

        for (int sample = 0; sample < sampleSize; sample++) {
            // 随机生成一个 2-Opt 邻居
            int[] currentRouteArray = this.currentPath.getPath(); // 获取当前路径数组
            int i, j;
            // 随机选择 i, j (确保有效 2-opt)
            do {
                i = random.nextInt(n); j = random.nextInt(n);
                if (i == j) continue; if (i > j) { int temp = i; i = j; j = temp; }
            } while (j == i + 1 || (i == 0 && j == n - 1));

            // 计算这个 2-opt 移动的 delta
            double delta = calculate2OptDelta(currentRouteArray, i, j);

            // 如果这个邻居是改进的，并且比当前找到的最好邻居还要好
            if (delta < bestDelta) {
                bestDelta = delta;
                // 生成这个邻居路径 (只在找到更好的改进时才实际生成)
                int[] neighborRoute = Arrays.copyOf(currentRouteArray, n);
                reverseSegment(neighborRoute, (i + 1) % n, j);
                bestNeighborFound = new Path(neighborRoute);
                bestNeighborEval = this.currentEvaluation + delta; // 预估新成本
            }
        }

        // 检查采样结果
        if (bestNeighborFound != null) {
            // 找到了改进邻居，移动过去
            this.currentPath = bestNeighborFound;
            this.currentEvaluation = bestNeighborEval; // 使用预估成本更新
            // 因为移动了，所以不触发变异
            this.needsMutation = false;

            // 验证一下预估成本（可选，但推荐）
            // double actualEval = this.evaluation.evaluate(this.currentPath);
            // if (Math.abs(actualEval - bestNeighborEval) > 1.0) {
            //    System.err.printf("Warning: Neighbor eval mismatch! Estimated: %.2f, Actual: %.2f%n", bestNeighborEval, actualEval);
            //    this.currentEvaluation = actualEval; // 修正
            // }

        } else {
            // 在抽样的邻居中没有找到改进，认为到达局部最优
            // 1. 更新全局最优解 (如果当前这个局部最优更好)
            if (this.currentEvaluation < this.bestEvaluation) {
                updateBestSolution(this.currentPath, this.currentEvaluation);
            }
            // 2. 设置标志，下次 loop 调用执行变异
            this.needsMutation = true;
        }
    }


    // --- 变异操作 ---
    /**
     * 对给定的路径应用指定次数的随机 Swap 操作作为变异/扰动。
     * @param path 要变异的路径
     * @param numSwaps 执行 Swap 的次数
     * @return 变异后的新 Path 对象
     */
    private Path applySwapMutation(Path path, int numSwaps) {
        int n = this.problem.getLength();
        if (n < 2) return path;

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


    // --- 辅助方法 ---
    private void updateBestSolution(Path path, double evaluation) {
        // 确保传入的 evaluation 是准确的 (最好是由 evaluate() 返回的)
        // 或者如果我们信任 delta 计算，可以传入 delta 计算的成本
        // 为保险起见，在更新全局最优时，最好用 evaluate() 核实一下
        double actualCost = this.evaluation.evaluate(path); // 获取准确成本
        // 如果实际成本确实优于记录的最佳成本
        if (actualCost < this.bestEvaluation - EPSILON) {
            this.bestPath = new Path(path); // 创建副本保存
            this.bestEvaluation = actualCost; // 使用准确成本更新
            System.out.printf("*** MHC found New Global Best: %.2f ***%n", this.bestEvaluation);
            // 注意：这里不需要再次调用 evaluate 了，因为上面已经调用过了
        }
        // 如果成本几乎一样，或者没有更好，则全局最优保持不变
    }

    private double calculate2OptDelta(int[] r, int i, int j) {
        int n = r.length; int a = r[i], b = r[(i + 1) % n], c = r[j], d = r[(j + 1) % n];
        return distances[a][c] + distances[b][d] - distances[a][b] - distances[c][d];
    }

    private void precomputeDistances(int n) { /* ... 同前 ... */
        this.distances = new double[n][n];
        for (int i = 0; i < n; i++) { Coordinates c1 = this.problem.getCoordinates(i);
            for (int j = i; j < n; j++) { if (i == j) this.distances[i][j] = 0;
            else { Coordinates c2 = this.problem.getCoordinates(j); double dist = c1.distance(c2);
                this.distances[i][j] = dist; this.distances[j][i] = dist; } } } }

    // 使用随机路径初始化，不再需要贪心了
    // private Path createGreedyPath(int n) { /* ... */ }

    private void reverseSegment(int[] route, int start, int end) { /* ... 同前 ... */
        int n = route.length; start = (start % n + n) % n; end = (end % n + n) % n; if (start == end) return;
        int p1 = start; int p2 = end; int segmentSize;
        if (p1 <= p2) { segmentSize = (p2 - p1 + 1) / 2; } else { segmentSize = (n - p1 + p2 + 1) / 2; }
        for (int k = 0; k < segmentSize; k++) { int temp = route[p1]; route[p1] = route[p2]; route[p2] = temp;
            p1 = (p1 + 1) % n; p2 = (p2 - 1 + n) % n; } }

    // 初始温度计算不再需要，因为这里不用 SA
    // private double calculateInitialTemperatureHeuristic(int n, double[][] distances) { /* ... */ }
}