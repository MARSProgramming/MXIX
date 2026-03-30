package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import dev.doglog.DogLog;

/**
 * Utilities for tuning and calibrating the vision system.
 *
 * <p>This class provides:
 * <ul>
 *   <li>Live tuning of vision constants via Shuffleboard</li>
 *   <li>Calibration tools for camera offset measurement</li>
 *   <li>Diagnostic tools for vision system health</li>
 *   <li>A/B testing for different vision configurations</li>
 * </ul>
 */
public class VisionTuningUtilities {

    // Tuning mode - enables live adjustment via dashboard
    private static boolean tuningMode = false;

    // A/B testing configurations
    public enum VisionConfig {
        /** Conservative configuration - high accuracy, slower updates */
        CONSERVATIVE("Conservative", 1.5, 0.3, 6.0),

        /** Balanced configuration - good accuracy and speed */
        BALANCED("Balanced", 1.0, 0.5, 8.0),

        /** Aggressive configuration - faster updates, more noise */
        AGGRESSIVE("Aggressive", 0.7, 0.7, 10.0),

        /** Custom configuration - user-defined */
        CUSTOM("Custom", 1.0, 0.5, 8.0);

        private final String name;
        private final double stdDevMultiplier;
        private final double confidenceThreshold;
        private final double maxTagDistance;

        VisionConfig(String name, double stdDevMultiplier, double confidenceThreshold, double maxTagDistance) {
            this.name = name;
            this.stdDevMultiplier = stdDevMultiplier;
            this.confidenceThreshold = confidenceThreshold;
            this.maxTagDistance = maxTagDistance;
        }

        public String getName() { return name; }
        public double getStdDevMultiplier() { return stdDevMultiplier; }
        public double getConfidenceThreshold() { return confidenceThreshold; }
        public double getMaxTagDistance() { return maxTagDistance; }
    }

    private static VisionConfig currentConfig = VisionConfig.BALANCED;

    /**
     * Enables or disables tuning mode.
     *
     * <p>When tuning mode is enabled, vision constants can be adjusted live via Shuffleboard.
     *
     * @param enabled Whether to enable tuning mode
     */
    public static void setTuningMode(boolean enabled) {
        tuningMode = enabled;

        if (enabled) {
            // Add tuning widgets to Shuffleboard
            SmartDashboard.putNumber("Vision/Tuning/StdDevMultiplier", VisionConstants.linearStdDevBaseline);
            SmartDashboard.putNumber("Vision/Tuning/ConfidenceThreshold", 0.5);
            SmartDashboard.putNumber("Vision/Tuning/MaxTagDistance", 6.5); // Default MAX_TAG_DIST value
            SmartDashboard.putNumber("Vision/Tuning/AngularVelocityScale", VisionConstants.angularVelocityStdDevScale);
            SmartDashboard.putNumber("Vision/Tuning/ZErrorScale", VisionConstants.zErrorStdDevScale);
            SmartDashboard.putString("Vision/Tuning/CurrentConfig", currentConfig.getName());

            DogLog.log("Vision/Tuning/Mode", true);
        } else {
            // Clear tuning values
            SmartDashboard.putNumber("Vision/Tuning/StdDevMultiplier", 0.0);
            SmartDashboard.putNumber("Vision/Tuning/ConfidenceThreshold", 0.0);
            SmartDashboard.putNumber("Vision/Tuning/MaxTagDistance", 0.0);
            DogLog.log("Vision/Tuning/Mode", false);
        }
    }

    /**
     * Gets whether tuning mode is enabled.
     *
     * @return True if tuning mode is enabled
     */
    public static boolean isTuningMode() {
        return tuningMode;
    }

    /**
     * Applies tuning values from Shuffleboard to vision constants.
     *
     * <p>Should be called periodically if tuning mode is enabled.
     */
    public static void applyTuningValues() {
        if (!tuningMode) {
            return;
        }

        double stdDevMult = SmartDashboard.getNumber("Vision/Tuning/StdDevMultiplier", VisionConstants.linearStdDevBaseline);
        double confThreshold = SmartDashboard.getNumber("Vision/Tuning/ConfidenceThreshold", 0.5);
        double maxDist = SmartDashboard.getNumber("Vision/Tuning/MaxTagDistance", 6.5);
        double angVelScale = SmartDashboard.getNumber("Vision/Tuning/AngularVelocityScale", VisionConstants.angularVelocityStdDevScale);
        double zErrorScale = SmartDashboard.getNumber("Vision/Tuning/ZErrorScale", VisionConstants.zErrorStdDevScale);

        // Apply tuning values (note: these are temporary and reset on code restart)
        DogLog.log("Vision/Tuning/AppliedStdDevMultiplier", stdDevMult);
        DogLog.log("Vision/Tuning/AppliedConfidenceThreshold", confThreshold);
        DogLog.log("Vision/Tuning/AppliedMaxTagDistance", maxDist);
        DogLog.log("Vision/Tuning/AppliedAngularVelocityScale", angVelScale);
        DogLog.log("Vision/Tuning/AppliedZErrorScale", zErrorScale);
    }

    /**
     * Sets the current vision configuration preset.
     *
     * @param config The configuration to apply
     */
    public static void setVisionConfig(VisionConfig config) {
        currentConfig = config;

        // Log the configuration change
        DogLog.log("Vision/Config/Name", config.getName());
        DogLog.log("Vision/Config/StdDevMultiplier", config.getStdDevMultiplier());
        DogLog.log("Vision/Config/ConfidenceThreshold", config.getConfidenceThreshold());
        DogLog.log("Vision/Config/MaxTagDistance", config.getMaxTagDistance());

        if (tuningMode) {
            SmartDashboard.putString("Vision/Tuning/CurrentConfig", config.getName());
        }
    }

    /**
     * Gets the current vision configuration.
     *
     * @return The current configuration
     */
    public static VisionConfig getVisionConfig() {
        return currentConfig;
    }

    /**
     * Calculates the optimal standard deviation for a given scenario.
     *
     * <p>This helper method provides recommendations for tuning vision constants.
     *
     * @param scenario The scenario description (e.g., "Fast motion", "Far from tags")
     * @return Recommended standard deviation multiplier
     */
    public static double getRecommendedStdDevMultiplier(String scenario) {
        switch (scenario.toLowerCase()) {
            case "fast motion":
            case "high speed":
                return 1.8; // Higher uncertainty during fast motion

            case "slow motion":
            case "precise":
                return 0.6; // Lower uncertainty during slow motion

            case "far from tags":
            case "poor visibility":
                return 2.0; // Higher uncertainty when far from tags

            case "close to tags":
            case "good visibility":
                return 0.4; // Lower uncertainty when close to tags

            case "auto":
            case "autonomous":
                return 0.8; // Moderate uncertainty for auto

            case "teleop":
            case "driver control":
                return 1.2; // Higher uncertainty for teleop (faster motion)

            default:
                return 1.0; // Default uncertainty
        }
    }

    /**
     * Creates a diagnostic report for the vision system.
     *
     * @param posesAccepted Number of poses accepted
     * @param posesRejected Number of poses rejected
     * @param averageConfidence Average confidence score
     * @param cameraHealth Health status of each camera
     * @return A formatted diagnostic string
     */
    public static String createDiagnosticReport(
            int posesAccepted,
            int posesRejected,
            double averageConfidence,
            int[] cameraHealth) {

        StringBuilder report = new StringBuilder();
        report.append("=== VISION SYSTEM DIAGNOSTIC REPORT ===\n");
        report.append("Configuration: ").append(currentConfig.getName()).append("\n");
        report.append("Tuning Mode: ").append(tuningMode ? "ENABLED" : "DISABLED").append("\n");
        report.append("\n");

        // Pose statistics
        int totalPoses = posesAccepted + posesRejected;
        double acceptanceRate = totalPoses > 0 ? (double) posesAccepted / totalPoses : 0.0;

        report.append("POSE STATISTICS:\n");
        report.append("  Total Observations: ").append(totalPoses).append("\n");
        report.append("  Accepted: ").append(posesAccepted).append("\n");
        report.append("  Rejected: ").append(posesRejected).append("\n");
        report.append("  Acceptance Rate: ").append(String.format("%.1f%%", acceptanceRate * 100)).append("\n");
        report.append("  Average Confidence: ").append(String.format("%.2f", averageConfidence)).append("\n");
        report.append("\n");

        // Health assessment
        report.append("HEALTH ASSESSMENT:\n");
        for (int i = 0; i < cameraHealth.length; i++) {
            String status;
            switch (cameraHealth[i]) {
                case 0:
                    status = "HEALTHY";
                    break;
                case 1:
                    status = "DEGRADED";
                    break;
                case 2:
                    status = "UNHEALTHY";
                    break;
                default:
                    status = "UNKNOWN";
            }
            report.append("  Camera ").append(i).append(": ").append(status).append("\n");
        }
        report.append("\n");

        // Recommendations
        report.append("RECOMMENDATIONS:\n");

        if (acceptanceRate < 0.5) {
            report.append("  ⚠ Low acceptance rate - consider increasing confidence threshold\n");
        } else if (acceptanceRate > 0.95) {
            report.append("  ⚠ Very high acceptance rate - consider decreasing confidence threshold\n");
        }

        if (averageConfidence < 0.3) {
            report.append("  ⚠ Low average confidence - check camera alignment and lighting\n");
        }

        boolean anyDegraded = false;
        for (int health : cameraHealth) {
            if (health >= 1) {
                anyDegraded = true;
                break;
            }
        }

        if (anyDegraded) {
            report.append("  ⚠ One or more cameras degraded - check camera connections\n");
        }

        if (acceptanceRate >= 0.7 && acceptanceRate <= 0.9 && averageConfidence >= 0.4) {
            report.append("  ✓ Vision system performing well\n");
        }

        report.append("\n");
        report.append("CURRENT SETTINGS:\n");
        report.append("  Std Dev Multiplier: ").append(String.format("%.2f", currentConfig.getStdDevMultiplier())).append("\n");
        report.append("  Confidence Threshold: ").append(String.format("%.2f", currentConfig.getConfidenceThreshold())).append("\n");
        report.append("  Max Tag Distance: ").append(String.format("%.1f m", currentConfig.getMaxTagDistance())).append("\n");

        return report.toString();
    }

    /**
     * Prints the diagnostic report to the console and logs to DogLog.
     *
     * <p>Note: System.out.println is used here intentionally for diagnostic output
     * to ensure visibility during troubleshooting and tuning.
     *
     * @param posesAccepted Number of poses accepted
     * @param posesRejected Number of poses rejected
     * @param averageConfidence Average confidence score
     * @param cameraHealth Health status of each camera
     */
    public static void printDiagnosticReport(
            int posesAccepted,
            int posesRejected,
            double averageConfidence,
            int[] cameraHealth) {

        String report = createDiagnosticReport(posesAccepted, posesRejected, averageConfidence, cameraHealth);

        // Print to console
        System.out.println(report);

        // Log to DogLog
        DogLog.log("Vision/Diagnostic/Report", report);
        DogLog.log("Vision/Diagnostic/AcceptanceRate", posesAccepted + posesRejected > 0 ?
            (double) posesAccepted / (posesAccepted + posesRejected) : 0.0);
        DogLog.log("Vision/Diagnostic/AverageConfidence", averageConfidence);
    }

    /**
     * Creates a calibration suggestion based on vision performance.
     *
     * @param currentPose Current vision pose estimate
     * @param odometryPose Current odometry pose estimate
     * @param visionTimestamp Timestamp of vision measurement
     * @return Calibration suggestion string
     */
    public static String createCalibrationSuggestion(Pose2d currentPose, Pose2d odometryPose, double visionTimestamp) {
        double distance = currentPose.getTranslation().getDistance(odometryPose.getTranslation());
        double rotationDiff = Math.abs(
            Math.toDegrees(currentPose.getRotation().getRadians() - odometryPose.getRotation().getRadians())
        );

        StringBuilder suggestion = new StringBuilder();
        suggestion.append("=== CALIBRATION SUGGESTION ===\n");
        suggestion.append("Vision-Odometry Difference:\n");
        suggestion.append(String.format("  Position: %.3f m\n", distance));
        suggestion.append(String.format("  Rotation: %.1f°\n", rotationDiff));

        if (distance > 0.5) {
            suggestion.append("  ⚠ Large position offset - consider recalibrating camera offset\n");
        } else if (distance > 0.2) {
            suggestion.append("  ⚠ Moderate position offset - may need camera adjustment\n");
        } else {
            suggestion.append("  ✓ Position offset within acceptable range\n");
        }

        if (rotationDiff > 10) {
            suggestion.append("  ⚠ Large rotation offset - consider recalibrating camera yaw\n");
        } else if (rotationDiff > 5) {
            suggestion.append("  ⚠ Moderate rotation offset - may need camera adjustment\n");
        } else {
            suggestion.append("  ✓ Rotation offset within acceptable range\n");
        }

        return suggestion.toString();
    }
}
