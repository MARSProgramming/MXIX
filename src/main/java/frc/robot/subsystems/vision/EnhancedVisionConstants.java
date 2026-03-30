package frc.robot.subsystems.vision;

/**
 * Enhanced constants for advanced vision processing.
 *
 * <p>This class contains additional tuning parameters for the advanced vision features:
 * <ul>
 *   <li>Pose prediction and latency compensation</li>
 *   <li>Multi-camera pose fusion</li>
 *   <li>Adaptive standard deviation scaling</li>
 *   <li>Outlier rejection thresholds</li>
 *   <li>Vision health monitoring</li>
 * </ul>
 */
public class EnhancedVisionConstants {

    // ===== POSE PREDICTION =====

    /**
     * Vision latency compensation time in seconds.
     * This accounts for the delay between when the camera captures an image
     * and when the robot code receives the pose estimate.
     */
    public static final double VISION_LATENCY_COMPENSATION_SECONDS = 0.05; // 50ms

    /**
     * Maximum time to predict pose forward in seconds.
     * Prevents unrealistic predictions during very long delays.
     */
    public static final double MAX_PREDICTION_TIME_SECONDS = 0.15; // 150ms

    // ===== MULTI-CAMERA FUSION =====

    /**
     * Minimum confidence threshold for multi-camera pose fusion.
     * Cameras below this confidence won't be included in fusion.
     */
    public static final double FUSION_MIN_CONFIDENCE = 0.4;

    /**
     * Minimum number of cameras required for pose fusion.
     * If fewer cameras meet the threshold, fusion won't be performed.
     */
    public static final int FUSION_MIN_CAMERAS = 2;

    /**
     * Maximum allowed distance between fused poses (meters).
     * If poses are too far apart, fusion won't be performed.
     */
    public static final double FUSION_MAX_POSE_DISTANCE = 0.5;

    // ===== OUTLIER REJECTION =====

    /**
     * Number of standard deviations for outlier rejection (3-sigma rule).
     * Poses beyond this threshold from the mean will be rejected.
     */
    public static final double OUTLIER_SIGMA_THRESHOLD = 3.0;

    /**
     * Number of recent poses to keep for statistical analysis.
     */
    public static final int POSE_HISTORY_SIZE = 10;

    /**
     * Maximum rotation difference for outlier detection (radians).
     * Poses with rotation difference beyond this threshold will be rejected.
     */
    public static final double OUTLIER_MAX_ROTATION_DIFFERENCE = Math.PI / 4; // 45 degrees

    // ===== ADAPTIVE STANDARD DEVIATION =====

    /**
     * Velocity multiplier for adaptive standard deviation.
     * Higher velocity increases pose uncertainty.
     */
    public static final double ADAPTIVE_VELOCITY_MULTIPLIER = 0.2;

    /**
     * Angular velocity multiplier for adaptive standard deviation.
     * Faster rotation increases pose uncertainty.
     */
    public static final double ADAPTIVE_ANGULAR_VELOCITY_MULTIPLIER = 0.3;

    /**
     * Camera health degradation multiplier.
     * Degraded cameras increase pose uncertainty.
     */
    public static final double ADAPTIVE_CAMERA_HEALTH_MULTIPLIER = 0.5;

    // ===== VISION HEALTH MONITORING =====

    /**
     * Interval between vision health checks (seconds).
     */
    public static final double HEALTH_CHECK_INTERVAL_SECONDS = 1.0;

    /**
     * Minimum acceptable pose acceptance rate (0.0 to 1.0).
     * Below this rate, a warning will be triggered.
     */
    public static final double HEALTH_MIN_ACCEPTANCE_RATE = 0.5;

    /**
     * Minimum acceptable average confidence (0.0 to 1.0).
     * Below this threshold, a warning will be triggered.
     */
    public static final double HEALTH_MIN_AVERAGE_CONFIDENCE = 0.3;

    /**
     * Minimum observations required before health checks are performed.
     */
    public static final int HEALTH_MIN_OBSERVATIONS = 10;

    // ===== CONFIDENCE CALCULATION =====

    /**
     * Distance decay factor for confidence calculation.
     * Confidence decreases as tag distance increases.
     */
    public static final double CONFIDENCE_DISTANCE_DECAY = 0.5;

    /**
     * Tag count factor for confidence calculation.
     * More tags increase confidence.
     */
    public static final double CONFIDENCE_TAG_COUNT_FACTOR = 3.0;

    /**
     * MegaTag 2 confidence bonus.
     * MegaTag 2 observations get this confidence boost.
     */
    public static final double CONFIDENCE_MEGATAG_2_BONUS = 0.2;

    /**
     * Motion penalty factor for confidence calculation.
     * Fast motion reduces confidence.
     */
    public static final double CONFIDENCE_MOTION_PENALTY = 0.1;

    // ===== SCENARIO-SPECIFIC TUNING =====

    /**
     * Standard deviation multiplier for fast motion scenarios.
     */
    public static final double SCENARIO_FAST_MOTION_MULTIPLIER = 1.8;

    /**
     * Standard deviation multiplier for slow motion scenarios.
     */
    public static final double SCENARIO_SLOW_MOTION_MULTIPLIER = 0.6;

    /**
     * Standard deviation multiplier when far from tags.
     */
    public static final double SCENARIO_FAR_FROM_TAGS_MULTIPLIER = 2.0;

    /**
     * Standard deviation multiplier when close to tags.
     */
    public static final double SCENARIO_CLOSE_TO_TAGS_MULTIPLIER = 0.4;

    /**
     * Standard deviation multiplier for autonomous mode.
     */
    public static final double SCENARIO_AUTONOMOUS_MULTIPLIER = 0.8;

    /**
     * Standard deviation multiplier for teleop mode.
     */
    public static final double SCENARIO_TELEOP_MULTIPLIER = 1.2;

    // ===== PERFORMANCE OPTIMIZATION =====

    /**
     * Maximum number of vision measurements to accept per second per camera.
     * Prevents flooding the Kalman filter with too many updates.
     */
    public static final int MAX_MEASUREMENTS_PER_SECOND = 30;

    /**
     * Minimum time between vision measurements from the same camera (seconds).
     */
    public static final double MIN_MEASUREMENT_INTERVAL_SECONDS = 1.0 / MAX_MEASUREMENTS_PER_SECOND;

    // ===== CALIBRATION =====

    /**
     * Threshold for camera offset recalibration (meters).
     * If vision-odometry difference exceeds this, recalibration is recommended.
     */
    public static final double CALIBRATION_POSITION_THRESHOLD = 0.5;

    /**
     * Threshold for camera rotation recalibration (degrees).
     * If rotation difference exceeds this, recalibration is recommended.
     */
    public static final double CALIBRATION_ROTATION_THRESHOLD = 10.0;

    /**
     * Acceptable camera offset range (meters).
     */
    public static final double CALIBRATION_ACCEPTABLE_OFFSET = 0.2;

    /**
     * Acceptable rotation offset range (degrees).
     */
    public static final double CALIBRATION_ACCEPTABLE_ROTATION = 5.0;

    // ===== DIAGNOSTICS =====

    /**
     * High acceptance rate warning threshold.
     * If acceptance rate is above this, poses may not be filtered enough.
     */
    public static final double DIAGNOSTIC_HIGH_ACCEPTANCE_RATE = 0.95;

    /**
     * Low acceptance rate warning threshold.
     * If acceptance rate is below this, poses may be over-filtered.
     */
    public static final double DIAGNOSTIC_LOW_ACCEPTANCE_RATE = 0.5;

    /**
     * Degraded camera confidence threshold.
     */
    public static final double DIAGNOSTIC_DEGRADED_CAMERA_CONFIDENCE = 0.5;

    /**
     * Unhealthy camera confidence threshold.
     */
    public static final double DIAGNOSTIC_UNHEALTHY_CAMERA_CONFIDENCE = 0.2;
}
