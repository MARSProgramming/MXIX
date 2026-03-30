package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import dev.doglog.DogLog;

/**
 * Advanced vision pose processor with multi-camera fusion and pose prediction.
 *
 * <p>This class provides enhanced vision processing capabilities:
 * <ul>
 *   <li>Pose prediction to compensate for vision latency during fast motion</li>
 *   <li>Multi-camera pose fusion using weighted averages</li>
 *   <li>Outlier rejection using statistical analysis</li>
 *   <li>Adaptive noise modeling based on conditions</li>
 *   <li>Vision health monitoring and diagnostics</li>
 * </ul>
 */
public class AdvancedVisionProcessor {

    // Vision latency compensation
    public static final double DEFAULT_VISION_LATENCY_SECONDS = 0.05; // 50ms typical Limelight latency

    // Multi-camera fusion
    private static final double FUSION_CONFIDENCE_THRESHOLD = 0.5; // Minimum confidence for fusion
    private static final int MIN_CAMERAS_FOR_FUSION = 2;

    // Outlier rejection
    private static final double OUTLIER_SIGMA_THRESHOLD = 3.0; // 3-sigma rule
    private static final int HISTORY_SIZE = 10; // Keep recent poses for statistical analysis

    // Health monitoring
    private static final double HEALTH_CHECK_INTERVAL_SECONDS = 1.0;
    private double lastHealthCheckTime = 0;

    private final double omegaSupplier;
    private final Translation2d velocitySupplier;

    // Pose history for statistical analysis
    private final PoseSnapshot[] poseHistory = new PoseSnapshot[HISTORY_SIZE];
    private int historyIndex = 0;

    // Vision health metrics
    private int totalPosesAccepted = 0;
    private int totalPosesRejected = 0;
    private double averageConfidence = 0.0;
    private int cameraHealthStatus[] = new int[2]; // 0 = healthy, 1 = degraded, 2 = unhealthy

    /**
     * Snapshot of a vision pose observation for statistical analysis.
     */
    private static class PoseSnapshot {
        final Pose2d pose;
        final double timestamp;
        final double confidence;
        final int cameraIndex;

        PoseSnapshot(Pose2d pose, double timestamp, double confidence, int cameraIndex) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.confidence = confidence;
            this.cameraIndex = cameraIndex;
        }
    }

    /**
     * Creates a new AdvancedVisionProcessor.
     *
     * @param omegaSupplier Supplier for robot angular velocity (rad/s)
     * @param velocityXSupplier Supplier for robot velocity X (m/s)
     * @param velocityYSupplier Supplier for robot velocity Y (m/s)
     */
    public AdvancedVisionProcessor(
            double omegaSupplier,
            double velocityXSupplier,
            double velocityYSupplier) {
        this.omegaSupplier = omegaSupplier;
        this.velocitySupplier = new Translation2d(velocityXSupplier, velocityYSupplier);
    }

    /**
     * Predicts robot pose after a time delta based on current velocity.
     *
     * <p>This compensates for vision latency by predicting where the robot will be.
     *
     * @param currentPose Current robot pose
     * @param deltaTimeSeconds Time to predict forward (seconds)
     * @return Predicted pose
     */
    public Pose2d predictPose(Pose2d currentPose, double deltaTimeSeconds) {
        if (deltaTimeSeconds <= 0) {
            return currentPose;
        }

        // Use default vision latency if not specified
        double predictionTime = deltaTimeSeconds;

        // Predict position change
        Translation2d positionChange = velocitySupplier.times(predictionTime);

        // Predict rotation change
        double rotationChange = omegaSupplier * predictionTime;
        Rotation2d predictedRotation = currentPose.getRotation().plus(new Rotation2d(rotationChange));

        // Combine into predicted pose
        Translation2d predictedTranslation = currentPose.getTranslation().plus(positionChange);

        // Handle angle wrapping for rotation
        predictedRotation = new Rotation2d(
            MathUtil.angleModulus(predictedRotation.getRadians())
        );

        return new Pose2d(predictedTranslation, predictedRotation);
    }

    /**
     * Calculates confidence score for a vision observation.
     *
     * <p>Confidence is based on:
     * <ul>
     *   <li>Tag distance (closer = higher confidence)</li>
     *   <li>Tag count (more tags = higher confidence)</li>
     *   <li>Observation type (MegaTag 2 > MegaTag 1)</li>
     *   <li>Robot motion (still = higher confidence)</li>
     * </ul>
     *
     * @param averageTagDistance Average distance to tags (meters)
     * @param tagCount Number of tags observed
     * @param isMegatag2 Whether this is a MegaTag 2 observation
     * @param angularVelocity Current angular velocity (rad/s)
     * @return Confidence score (0.0 to 1.0)
     */
    public double calculateConfidence(
            double averageTagDistance,
            int tagCount,
            boolean isMegatag2,
            double angularVelocity) {

        // Distance factor: 1.0 at 1m, 0.5 at 3m, 0.2 at 6m
        double distanceFactor = 1.0 / (1.0 + averageTagDistance * 0.5);

        // Tag count factor: more tags = higher confidence
        double tagCountFactor = Math.min(1.0, tagCount / 3.0);

        // MegaTag 2 bonus
        double megatagBonus = isMegatag2 ? 0.2 : 0.0;

        // Motion penalty: fast motion reduces confidence
        double motionPenalty = Math.min(0.3, Math.abs(angularVelocity) * 0.1);

        // Combine factors
        double confidence = distanceFactor * 0.4 + tagCountFactor * 0.3 + megatagBonus - motionPenalty;

        // Clamp to valid range
        return MathUtil.clamp(confidence, 0.0, 1.0);
    }

    /**
     * Fuses multiple camera observations into a single pose estimate.
     *
     * <p>Uses weighted average based on confidence scores.
     *
     * @param poses Array of pose observations from different cameras
     * @param confidences Array of confidence scores for each pose
     * @return Fused pose estimate, or null if fusion not possible
     */
    public Pose2d fusePoses(Pose2d[] poses, double[] confidences) {
        if (poses.length != confidences.length || poses.length < MIN_CAMERAS_FOR_FUSION) {
            return null;
        }

        // Check if all cameras meet confidence threshold
        boolean allConfident = true;
        for (double confidence : confidences) {
            if (confidence < FUSION_CONFIDENCE_THRESHOLD) {
                allConfident = false;
                break;
            }
        }

        if (!allConfident) {
            return null;
        }

        // Calculate weighted average
        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedCos = 0.0;
        double weightedSin = 0.0;

        for (int i = 0; i < poses.length; i++) {
            double weight = confidences[i];
            totalWeight += weight;

            weightedX += poses[i].getX() * weight;
            weightedY += poses[i].getY() * weight;

            // Handle rotation with trigonometry to avoid angle wrapping issues
            weightedCos += Math.cos(poses[i].getRotation().getRadians()) * weight;
            weightedSin += Math.sin(poses[i].getRotation().getRadians()) * weight;
        }

        // Normalize weights
        weightedX /= totalWeight;
        weightedY /= totalWeight;
        weightedCos /= totalWeight;
        weightedSin /= totalWeight;

        // Calculate fused rotation
        Rotation2d fusedRotation = new Rotation2d(Math.atan2(weightedSin, weightedCos));

        return new Pose2d(weightedX, weightedY, fusedRotation);
    }

    /**
     * Performs statistical outlier rejection on vision observations.
     *
     * <p>Uses a 3-sigma rule to reject poses that are statistically unlikely.
     *
     * @param pose Candidate pose to check
     * @param meanPose Mean of recent poses
     * @param stdDev Standard deviation of recent poses
     * @return True if pose should be rejected as an outlier
     */
    public boolean isOutlier(Pose2d pose, Pose2d meanPose, double stdDev) {
        // Calculate distance from mean
        double distance = pose.getTranslation().getDistance(meanPose.getTranslation());

        // Check if beyond 3-sigma threshold
        if (distance > stdDev * OUTLIER_SIGMA_THRESHOLD) {
            DogLog.log("Vision/OutlierRejected", true);
            DogLog.log("Vision/OutlierDistance", distance);
            return true;
        }

        // Check rotation difference
        double rotationDiff = Math.abs(
            MathUtil.angleModulus(pose.getRotation().getRadians() - meanPose.getRotation().getRadians())
        );

        // Reject if rotation difference is too large (> 45 degrees)
        if (rotationDiff > Math.PI / 4) {
            DogLog.log("Vision/RotationOutlierRejected", true);
            return true;
        }

        return false;
    }

    /**
     * Calculates statistics on recent vision poses for outlier detection.
     *
     * @return Array containing [meanX, meanY, meanRotation, stdDev]
     */
    public double[] calculatePoseStatistics() {
        int validCount = 0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumCos = 0.0;
        double sumSin = 0.0;

        // Calculate mean
        for (int i = 0; i < HISTORY_SIZE; i++) {
            if (poseHistory[i] != null) {
                sumX += poseHistory[i].pose.getX();
                sumY += poseHistory[i].pose.getY();
                sumCos += Math.cos(poseHistory[i].pose.getRotation().getRadians());
                sumSin += Math.sin(poseHistory[i].pose.getRotation().getRadians());
                validCount++;
            }
        }

        if (validCount == 0) {
            return new double[]{0, 0, 0, Double.MAX_VALUE};
        }

        double meanX = sumX / validCount;
        double meanY = sumY / validCount;
        double meanRotation = Math.atan2(sumSin, sumCos);

        // Calculate standard deviation
        double sumSquaredDiff = 0.0;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            if (poseHistory[i] != null) {
                double dx = poseHistory[i].pose.getX() - meanX;
                double dy = poseHistory[i].pose.getY() - meanY;
                sumSquaredDiff += dx * dx + dy * dy;
            }
        }

        double stdDev = Math.sqrt(sumSquaredDiff / validCount);

        return new double[]{meanX, meanY, meanRotation, stdDev};
    }

    /**
     * Adds a pose to the history for statistical analysis.
     *
     * @param pose Pose to add
     * @param timestamp Timestamp of the pose
     * @param confidence Confidence score (0.0 to 1.0)
     * @param cameraIndex Index of the camera that provided the pose
     */
    public void addPoseToHistory(Pose2d pose, double timestamp, double confidence, int cameraIndex) {
        poseHistory[historyIndex] = new PoseSnapshot(pose, timestamp, confidence, cameraIndex);
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    }

    /**
     * Performs periodic health checks on the vision system.
     *
     * <p>Checks camera health, acceptance rates, and overall system performance.
     */
    public void performHealthCheck() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (currentTime - lastHealthCheckTime < HEALTH_CHECK_INTERVAL_SECONDS) {
            return;
        }

        lastHealthCheckTime = currentTime;

        // Calculate acceptance rate
        int totalObservations = totalPosesAccepted + totalPosesRejected;
        double acceptanceRate = totalObservations > 0 ?
            (double) totalPosesAccepted / totalObservations : 0.0;

        // Log health metrics
        DogLog.log("Vision/Health/AcceptanceRate", acceptanceRate);
        DogLog.log("Vision/Health/AcceptedCount", totalPosesAccepted);
        DogLog.log("Vision/Health/RejectedCount", totalPosesRejected);
        DogLog.log("Vision/Health/AverageConfidence", averageConfidence);

        // Check for vision issues
        if (acceptanceRate < 0.5 && totalObservations > 10) {
            DogLog.log("Vision/Health/Warning", "Low acceptance rate - check camera alignment");
        }

        if (averageConfidence < 0.3 && totalObservations > 10) {
            DogLog.log("Vision/Health/Warning", "Low confidence - check lighting/tag visibility");
        }
    }

    /**
     * Updates vision health statistics.
     *
     * @param accepted Whether the pose was accepted
     * @param confidence Confidence score of the pose
     * @param cameraIndex Index of the camera
     */
    public void updateHealthStats(boolean accepted, double confidence, int cameraIndex) {
        if (accepted) {
            totalPosesAccepted++;
        } else {
            totalPosesRejected++;
        }

        // Update running average confidence
        int totalCount = totalPosesAccepted + totalPosesRejected;
        averageConfidence = (averageConfidence * (totalCount - 1) + confidence) / totalCount;

        // Update camera health status
        if (confidence > 0.5) {
            cameraHealthStatus[cameraIndex] = 0; // Healthy
        } else if (confidence > 0.2) {
            cameraHealthStatus[cameraIndex] = 1; // Degraded
        } else {
            cameraHealthStatus[cameraIndex] = 2; // Unhealthy
        }

        DogLog.log("Vision/Health/Camera" + cameraIndex + "Status", cameraHealthStatus[cameraIndex]);
    }

    /**
     * Resets health statistics.
     */
    public void resetHealthStats() {
        totalPosesAccepted = 0;
        totalPosesRejected = 0;
        averageConfidence = 0.0;
        for (int i = 0; i < cameraHealthStatus.length; i++) {
            cameraHealthStatus[i] = 0;
        }
    }

    /**
     * Gets the recommended standard deviation multiplier based on conditions.
     *
     * <p>Adjusts vision uncertainty based on:
     * <ul>
     *   <li>Robot velocity (faster = more uncertainty)</li>
     *   <li>Angular velocity (faster rotation = more uncertainty)</li>
     *   <li>Camera health (degraded = more uncertainty)</li>
     * </ul>
     *
     * @param cameraIndex Index of the camera
     * @param baseStdDev Base standard deviation
     * @return Adjusted standard deviation
     */
    public double getAdaptiveStdDev(int cameraIndex, double baseStdDev) {
        double velocity = velocitySupplier.getNorm();
        double angularVel = Math.abs(omegaSupplier);

        // Velocity penalty: more speed = more uncertainty
        double velocityMultiplier = 1.0 + velocity * 0.2;

        // Angular velocity penalty: faster rotation = more uncertainty
        double angularMultiplier = 1.0 + angularVel * 0.3;

        // Camera health penalty
        double healthMultiplier = 1.0 + cameraHealthStatus[cameraIndex] * 0.5;

        return baseStdDev * velocityMultiplier * angularMultiplier * healthMultiplier;
    }
}
