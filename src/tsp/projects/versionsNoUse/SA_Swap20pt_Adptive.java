// -------------  Competitor Adaptive Simulated-Annealing for TSP  -------------
//  此版本移除了 evaluation.getRemainingTime() 依赖，改用固定的总运行时长常量。
//  关键特性与上一版相同：
//  1.  自适应初温（目标接受率 70%）
//  2.  动态冷却率（按阶段计算 α）
//  3.  NUM_PHASES 段退火 + 大扰动重启 + 快速 2‑opt 精化
//  4.  Swap 概率随温度线性上升
//  5.  扰动规模随问题规模自适应
// -----------------------------------------------------------------------------
package tsp.projects.versionsNoUse;

import java.util.Random;
import tsp.evaluation.*;
import tsp.projects.*;

public class SA_Swap20pt_Adptive extends DemoProject {

    // ------------ 全局可调常量 ----------------
    private static final double  TARGET_ACCEPT_RATE = 0.70;   // 初始劣解接受率
    private static final double  FINAL_TEMP_FACTOR  = 1E-4;   // 末温 = 初温 * 1E-4
    private static final int     NUM_PHASES         = 3;      // 分段数
    private static final int     SAMPLE_MOVES       = 400;    // 估计 ΔE 采样数
    private static final double  SWAP_PROB_LO       = 0.10;   // 高温 Swap 概率
    private static final double  SWAP_PROB_HI       = 0.30;   // 低温 Swap 概率
    private static final long    TOTAL_RUNTIME_MS   = 58_000; // 总运行 58 秒, 留 2 秒收尾

    // ------------- 运行时成员 ----------------
    private final Random rng = new Random();
    private double        T, T0, T_end, alpha;  // 温度参数
    private int           phase = 0;            // 当前阶段 idx
    private long          phaseStartTime;       // 当前阶段开始时间
    private long          phaseDurationMs;      // 每阶段时长

    private Path   curPath, bestPath;
    private double curEval,  bestEval;
    private double[][] dist;

    // ------- 构造 --------
    public SA_Swap20pt_Adptive(Evaluation evaluation) throws InvalidProjectException {
        super(evaluation);
        addAuthor("SAGANG TANWOUO Achille & SONG Lewei");
        setMethodName("SA_Swap2Opt_Adaptive");
    }

    // ---------------- 初始化 ----------------
    @Override
    public void initialization() {
        final int n = problem.getLength();
        precomputeDistances(n);
        curPath  = createGreedyPath(n);
        curEval  = evaluation.evaluate(curPath);
        bestPath = new Path(curPath);
        bestEval = curEval;

        // --- 自适应初温 ---
        double avgPosDelta = estimateAverageDelta(n);
        T0   = -avgPosDelta / Math.log(TARGET_ACCEPT_RATE);
        if (T0 < 1e-6) T0 = 1.0; // 防护
        T    = T0;
        T_end = T0 * FINAL_TEMP_FACTOR;

        // --- 时间阶段划分 ---
        phaseDurationMs = TOTAL_RUNTIME_MS / NUM_PHASES;
        phaseStartTime  = System.currentTimeMillis();
        computeCoolingRate();
    }

    // ---------------- 主循环 ----------------
    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        // 若阶段结束且还有阶段剩余
        if (now - phaseStartTime >= phaseDurationMs && phase < NUM_PHASES - 1) {
            phase++;
            largePerturbation();
            local2Opt(curPath);
            T  = T0;                       // 升温
            phaseStartTime = now;
            computeCoolingRate();
        }

        saIteration();
        coolDown();
    }

    // ---------------- 关键方法 ----------------
    private void saIteration() {
        int n = problem.getLength();
        int[] route = curPath.getCopyPath();
        double progress = 1 - (T - T_end) / (T0 - T_end);
        double pSwap = SWAP_PROB_LO + (SWAP_PROB_HI - SWAP_PROB_LO) * progress;

        boolean doSwap = rng.nextDouble() < pSwap;
        int i, j; double delta;
        if (doSwap || n < 4) {
            // Swap
            i = rng.nextInt(n);
            do { j = rng.nextInt(n); } while (j == i);
            delta = swapDelta(route, i, j);
            if (accept(delta)) {
                int tmp = route[i]; route[i] = route[j]; route[j] = tmp;
                acceptNeighbor(route, delta);
            }
        } else {
            // 2‑opt
            do {
                i = rng.nextInt(n); j = rng.nextInt(n);
                if (i > j) { int t = i; i = j; j = t; }
            } while (j == i || j == i + 1 || (i == 0 && j == n - 1));
            delta = twoOptDelta(route, i, j);
            if (accept(delta)) {
                reverseSegment(route, (i + 1) % n, j);
                acceptNeighbor(route, delta);
            }
        }
    }

    private boolean accept(double delta) {
        return delta < 0 || Math.exp(-delta / T) > rng.nextDouble();
    }

    private void acceptNeighbor(int[] newRoute, double delta) {
        curPath = new Path(newRoute);
        curEval += delta;
        if (curEval < bestEval) { bestPath = curPath; bestEval = curEval; }
    }

    private void coolDown() {
        T *= alpha;
        if (T < T_end) T = T_end;
    }

    // -------- 初温采样 --------
    private double estimateAverageDelta(int n) {
        double sum = 0; int cnt = 0;
        int[] route = curPath.getCopyPath();
        for (int s = 0; s < SAMPLE_MOVES; s++) {
            boolean swap = rng.nextBoolean();
            int i = rng.nextInt(n), j;
            do { j = rng.nextInt(n); } while (i == j);
            double d;
            if (swap || n < 4) d = swapDelta(route, i, j);
            else {
                if (i > j) { int t = i; i = j; j = t; }
                if (j == i + 1 || (i == 0 && j == n - 1)) continue;
                d = twoOptDelta(route, i, j);
            }
            if (d > 0) { sum += d; cnt++; }
        }
        return cnt == 0 ? 1.0 : sum / cnt;
    }

    // -------- 大扰动 --------
    private void largePerturbation() {
        int n = problem.getLength();
        double minRatio = n <= 100 ? 0.10 : (n <= 300 ? 0.15 : 0.20);
        double maxRatio = n <= 100 ? 0.25 : 0.50;
        int minLen = Math.max(2, (int)(n * minRatio));
        int maxLen = Math.min(n - 2, (int)(n * maxRatio));
        int i, j;
        do {
            i = rng.nextInt(n); j = rng.nextInt(n);
            if (i > j) { int t = i; i = j; j = t; }
        } while (j - i < minLen || j - i > maxLen || (i == 0 && j == n - 1));
        int[] r = curPath.getCopyPath();
        reverseSegment(r, i + 1, j);
        curPath = new Path(r);
        curEval = evaluation.evaluate(curPath);
    }

    // -------- 计算冷却率 --------
    private void computeCoolingRate() {
        long estSteps = phaseDurationMs / 2; // 经验：一次循环大约 2ms
        if (estSteps < 1) estSteps = 1;
        alpha = Math.pow(T_end / T0, 1.0 / estSteps);
        if (alpha > 0.9999) alpha = 0.9999;
    }

    // -------- Δ 计算 --------
    private double twoOptDelta(int[] r, int i, int j) {
        int n = r.length;
        int a = r[i], b = r[(i + 1) % n];
        int c = r[j], d = r[(j + 1) % n];
        return dist[a][c] + dist[b][d] - dist[a][b] - dist[c][d];
    }
    private double swapDelta(int[] r, int i, int j) {
        int n = r.length;
        int a = r[i], b = r[j];
        int ap = r[(i - 1 + n) % n], an = r[(i + 1) % n];
        int bp = r[(j - 1 + n) % n], bn = r[(j + 1) % n];
        if (i == (j + 1) % n || j == (i + 1) % n) {
            double removed = dist[ap][a] + dist[a][b] + dist[b][bn];
            double added   = dist[ap][b] + dist[b][a] + dist[a][bn];
            return added - removed;
        }
        double removed = dist[ap][a] + dist[a][an] + dist[bp][b] + dist[b][bn];
        double added   = dist[ap][b] + dist[b][an] + dist[bp][a] + dist[a][bn];
        return added - removed;
    }

    private void local2Opt(Path p) {
        int n = problem.getLength();
        int[] r = p.getCopyPath();
        boolean improved;
        do {
            improved = false;
            outer:
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    if (i == 0 && j == n - 1) continue;
                    double d = twoOptDelta(r, i, j);
                    if (d < -1E-9) {
                        reverseSegment(r, i + 1, j);
                        improved = true;
                        break outer;
                    }
                }
            }
        } while (improved);
        curPath = new Path(r);
        curEval = evaluation.evaluate(curPath);
        if (curEval < bestEval) { bestPath = curPath; bestEval = curEval; }
    }

    // ================= 工具函数 =================
    private void precomputeDistances(int n) {
        dist = new double[n][n];
        for (int i = 0; i < n; i++) {
            Coordinates c1 = problem.getCoordinates(i);
            for (int j = i; j < n; j++) {
                double d = (i == j) ? 0 : c1.distance(problem.getCoordinates(j));
                dist[i][j] = d; dist[j][i] = d;
            }
        }
    }

    private Path createGreedyPath(int n) {
        boolean[] vis = new boolean[n];
        int[] r = new int[n];
        int cur = rng.nextInt(n); r[0] = cur; vis[cur] = true;
        for (int idx = 1; idx < n; idx++) {
            int next = -1; double best = Double.MAX_VALUE;
            for (int v = 0; v < n; v++) if (!vis[v] && dist[cur][v] < best) { best = dist[cur][v]; next = v; }
            r[idx] = next; vis[next] = true; cur = next;
        }
        return new Path(r);
    }

    private void reverseSegment(int[] r, int i, int j) {
        int n = r.length;
        while (i != j && (i + n - 1) % n != j) {
            int tmp = r[i]; r[i] = r[j]; r[j] = tmp;
            i = (i + 1) % n; j = (j - 1 + n) % n;
        }
    }
}
