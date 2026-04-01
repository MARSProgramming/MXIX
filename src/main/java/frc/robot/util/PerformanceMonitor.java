package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Utility class for monitoring robot performance metrics.
 *
 * <p>This class provides utilities to track:
 * <ul>
 *   <li>Loop timing - detects when periodic loops take too long</li>
 *   <li>CPU usage - monitors robot processor utilization</li>
 *   <li>Memory usage - tracks Java heap memory usage</li>
 *   <li>Bandwidth - logs NetworkTables and CAN bus usage</li>
 * </ul>
 *
 * <p>Performance metrics are logged to DogLog for analysis in AdvantageKit.
 */
public class PerformanceMonitor {

    // Loop timing thresholds
    private static final double LOOP_WARNING_THRESHOLD_MS = 22.0; // 10% over 20ms nominal
    private static final double LOOP_CRITICAL_THRESHOLD_MS = 25.0; // 25% over 20ms nominal
    private static final double LOOP_OVERRUN_ALERT_THRESHOLD_MS = 30.0; // 50% over 20ms nominal

    // Memory thresholds
    private static final double MEMORY_WARNING_THRESHOLD_MB = 450.0; // Near 512MB limit
    private static final double MEMORY_CRITICAL_THRESHOLD_MB = 480.0; // Very close to limit

    private final Alert loopTimingAlert;
    private final Alert memoryUsageAlert;
    private final Alert cpuUsageAlert;

    private int loopOverrunCount = 0;


    /**
     * Creates a new PerformanceMonitor.
     */
    public PerformanceMonitor() {
        loopTimingAlert = new Alert(
            "Loop timing degraded - check Driver Station Log for details",
            AlertType.kWarning);
        memoryUsageAlert = new Alert(
            "High memory usage - robot code may crash",
            AlertType.kError);
        cpuUsageAlert = new Alert(
            "High CPU usage - robot performance degraded",
            AlertType.kWarning);
    }

    /**
     * Checks loop timing and logs performance metrics.
     *
     * <p>Should be called once per robot periodic loop.
     *
     * @param loopTimeMs The time taken for the last loop iteration in milliseconds
     */
    public void checkLoopTiming(double loopTimeMs) {
        // Log loop time
        DogLog.log("Performance/LoopTimeMs", loopTimeMs);

        // Check for loop overruns
        if (loopTimeMs > LOOP_OVERRUN_ALERT_THRESHOLD_MS) {
            loopOverrunCount++;
            loopTimingAlert.set(true);

            // Log critical overrun
            DogLog.log("Performance/LoopOverrunCritical", true);
        } else if (loopTimeMs > LOOP_CRITICAL_THRESHOLD_MS) {
            loopOverrunCount++;

            // Log degraded performance
            DogLog.log("Performance/LoopOverrunWarning", true);
        } else if (loopTimeMs > LOOP_WARNING_THRESHOLD_MS) {
            // Log warning but don't alert
            DogLog.log("Performance/LoopOverrunMinor", true);
            loopOverrunCount++;
            loopTimingAlert.set(true);

            // Log critical overrun
            DogLog.log("Performance/LoopOverrunCritical", true);
        } else if (loopTimeMs > LOOP_CRITICAL_THRESHOLD_MS) {
            loopOverrunCount++;

            // Log degraded performance
            DogLog.log("Performance/LoopOverrunWarning", true);
        } else if (loopTimeMs > LOOP_WARNING_THRESHOLD_MS) {
            // Log warning but don't alert
            DogLog.log("Performance/LoopOverrunMinor", true);
        } else {
            // Reset counters if performance is good
            if (loopOverrunCount > 0) {
                loopOverrunCount--;
            }
            if (loopOverrunCount == 0) {
                loopTimingAlert.set(false);
            }

            // Clear overrun flags
            DogLog.log("Performance/LoopOverrunCritical", false);
            DogLog.log("Performance/LoopOverrunWarning", false);
            DogLog.log("Performance/LoopOverrunMinor", false);
        }

        DogLog.log("Performance/LoopOverrunCount", loopOverrunCount);
    }

    /**
     * Checks memory usage and logs metrics.
     *
     * <p>Should be called periodically (e.g., once per second) to monitor memory.
     */
    public void checkMemoryUsage() {
        Runtime runtime = Runtime.getRuntime();

        // Get memory usage in MB
        double totalMemoryMB = runtime.totalMemory() / (1024.0 * 1024.0);
        double freeMemoryMB = runtime.freeMemory() / (1024.0 * 1024.0);
        double usedMemoryMB = totalMemoryMB - freeMemoryMB;
        double maxMemoryMB = runtime.maxMemory() / (1024.0 * 1024.0);
        double memoryUsagePercent = (usedMemoryMB / maxMemoryMB) * 100.0;

        // Log memory metrics
        DogLog.log("Performance/MemoryUsedMB", usedMemoryMB);
        DogLog.log("Performance/MemoryFreeMB", freeMemoryMB);
        DogLog.log("Performance/MemoryTotalMB", totalMemoryMB);
        DogLog.log("Performance/MemoryMaxMB", maxMemoryMB);
        DogLog.log("Performance/MemoryUsagePercent", memoryUsagePercent);

        // Check for high memory usage
        if (usedMemoryMB > MEMORY_CRITICAL_THRESHOLD_MB) {
            memoryUsageAlert.set(true);
            DogLog.log("Performance/MemoryCritical", true);

            // Suggest garbage collection
            System.gc();
        } else if (usedMemoryMB > MEMORY_WARNING_THRESHOLD_MB) {
            DogLog.log("Performance/MemoryWarning", true);
        } else {
            memoryUsageAlert.set(false);
            DogLog.log("Performance/MemoryCritical", false);
            DogLog.log("Performance/MemoryWarning", false);
        }
    }

    /**
     * Checks and logs CPU performance metrics.
     *
     * <p>Note: Java doesn't provide direct CPU usage access.
     * This method logs available CPU-related metrics.
     */
    public void checkCpuUsage() {
        // Get number of available processors
        int availableProcessors = Runtime.getRuntime().availableProcessors();

        // Log CPU metrics
        DogLog.log("Performance/AvailableProcessors", availableProcessors);

        // Check if we're running in simulation (uses more CPU)
        boolean isSimulation = edu.wpi.first.wpilibj.RobotBase.isSimulation();
        DogLog.log("Performance/IsSimulation", isSimulation);

        if (isSimulation && availableProcessors < 4) {
            cpuUsageAlert.set(true);
        } else {
            cpuUsageAlert.set(false);
        }
    }

    /**
     * Logs network performance metrics.
     *
     * <p>Should be called periodically to monitor NetworkTables bandwidth.
     *
     * @param networkTablesLatencyMs NetworkTables latency in milliseconds
     */
    public void logNetworkMetrics(double networkTablesLatencyMs) {
        DogLog.log("Performance/NetworkTablesLatencyMs", networkTablesLatencyMs);

        // Alert on high latency
        if (networkTablesLatencyMs > 50.0) {
            DogLog.log("Performance/NetworkLatencyHigh", true);
        } else {
            DogLog.log("Performance/NetworkLatencyHigh", false);
        }
    }

    /**
     * Resets all alert states.
     *
     * <p>Useful when resetting between matches or modes.
     */
    public void resetAlerts() {
        loopOverrunCount = 0;
        loopTimingAlert.set(false);
        memoryUsageAlert.set(false);
        cpuUsageAlert.set(false);
    }

    /**
     * Creates a simple loop timing tracker.
     *
     * <p>Usage:
     * <pre>
     * LoopTimer timer = new LoopTimer();
     * timer.start();
     * // ... code to measure ...
     * timer.stop();
     * double elapsedMs = timer.getElapsedMs();
     * </pre>
     */
    public static class LoopTimer {
        private double startTime;
        private double endTime;

        /**
         * Starts the timer.
         */
        public void start() {
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }

        /**
         * Stops the timer.
         */
        public void stop() {
            endTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }

        /**
         * Gets the elapsed time in milliseconds.
         *
         * @return Elapsed time in milliseconds
         */
        public double getElapsedMs() {
            return (endTime - startTime) * 1000.0;
        }

        /**
         * Gets the elapsed time in seconds.
         *
         * @return Elapsed time in seconds
         */
        public double getElapsedSeconds() {
            return endTime - startTime;
        }
    }

    /**
     * Creates a scoped timer that automatically stops when closed.
     *
     * <p>Usage with try-with-resources:
     * <pre>
     * try (var timer = new ScopedTimer("MyOperation")) {
     *     // ... code to measure ...
     * }
     * // Timer automatically stops and logs
     * </pre>
     */
    public static class ScopedTimer implements AutoCloseable {
        private final String name;
        private final double startTime;

        /**
         * Creates a new scoped timer.
         *
         * @param name The name to use when logging the elapsed time
         */
        public ScopedTimer(String name) {
            this.name = name;
            this.startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }

        @Override
        public void close() {
            double elapsedMs = (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime) * 1000.0;
            DogLog.log("Performance/ScopedTimer/" + name + "Ms", elapsedMs);

            // Warn if operation took too long
            if (elapsedMs > 5.0) {
                DogLog.log("Performance/ScopedTimer/" + name + "Slow", true);
            }
        }
    }
}
