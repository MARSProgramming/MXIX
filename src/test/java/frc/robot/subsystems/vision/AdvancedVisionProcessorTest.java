package frc.robot.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Unit tests for AdvancedVisionProcessor.
 * Tests pose prediction, multi-camera fusion, outlier rejection, and confidence scoring.
 */
public class AdvancedVisionProcessorTest {

    private AdvancedVisionProcessor processor;
    private static final double TEST_OMEGA = 0.5; // rad/s
    private static final double TEST_VELOCITY_X = 1.0; // m/s
    private static final double TEST_VELOCITY_Y = 0.5; // m/s

    @BeforeEach
    public void setUp() {
        processor = new AdvancedVisionProcessor(
            TEST_OMEGA,
            TEST_VELOCITY_X,
            TEST_VELOCITY_Y
        );
    }

    /**
     * Test pose prediction with zero velocity.
     * Pose should remain unchanged.
     */
    @Test
    public void testPosePredictionZeroVelocity() {
        // Create processor with zero velocity
        AdvancedVisionProcessor zeroVelProcessor = new AdvancedVisionProcessor(
            0.0, // No rotation
            0.0, // No velocity X
            0.0  // No velocity Y
        );

        Pose2d originalPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        Pose2d predictedPose = zeroVelProcessor.predictPose(originalPose, 0.05);

        // With zero velocity, pose shouldn't change
        assertNotNull(predictedPose);
        assertEquals(originalPose.getX(), predictedPose.getX(), 0.001);
        assertEquals(originalPose.getY(), predictedPose.getY(), 0.001);
    }

    /**
     * Test pose prediction with forward velocity.
     */
    @Test
    public void testPosePredictionForwardVelocity() {
        AdvancedVisionProcessor movingProcessor = new AdvancedVisionProcessor(
            0.0, // No rotation
            2.0, // 2 m/s forward
            0.0  // No sideways velocity
        );

        Pose2d originalPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        double deltaTime = 0.1; // 100ms

        Pose2d predictedPose = movingProcessor.predictPose(originalPose, deltaTime);

        // Should move forward by 0.2m (2.0 m/s * 0.1s)
        assertEquals(0.2, predictedPose.getX(), 0.01);
        assertEquals(0.0, predictedPose.getY(), 0.01);
        assertEquals(0.0, predictedPose.getRotation().getDegrees(), 0.01);
    }

    /**
     * Test pose prediction with rotation.
     */
    @Test
    public void testPosePredictionWithRotation() {
        AdvancedVisionProcessor rotatingProcessor = new AdvancedVisionProcessor(
            Math.PI / 2, // 90 degrees/sec
            0.0,        // No forward velocity
            0.0         // No sideways velocity
        );

        Pose2d originalPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        double deltaTime = 0.5; // 500ms

        Pose2d predictedPose = rotatingProcessor.predictPose(originalPose, deltaTime);

        // Should rotate 45 degrees (90 deg/s * 0.5s)
        assertEquals(45.0, predictedPose.getRotation().getDegrees(), 1.0);
    }

    /**
     * Test confidence calculation with good conditions.
     */
    @Test
    public void testConfidenceCalculationGoodConditions() {
        double confidence = processor.calculateConfidence(
            1.0,    // 1 meter from tags (close)
            3,      // 3 tags seen
            true,   // MegaTag 2
            0.1     // Slow rotation (0.1 rad/s)
        );

        // Should have high confidence (> 0.6)
        assertTrue(confidence > 0.6, "Good conditions should produce high confidence");
        assertTrue(confidence <= 1.0, "Confidence should not exceed 1.0");
    }

    /**
     * Test confidence calculation with poor conditions.
     */
    @Test
    public void testConfidenceCalculationPoorConditions() {
        double confidence = processor.calculateConfidence(
            5.0,    // 5 meters from tags (far)
            1,      // Only 1 tag seen
            false,  // MegaTag 1
            3.0     // Fast rotation (3 rad/s)
        );

        // Should have low confidence (< 0.4)
        assertTrue(confidence < 0.4, "Poor conditions should produce low confidence");
        assertTrue(confidence >= 0.0, "Confidence should not be negative");
    }

    /**
     * Test confidence calculation with average conditions.
     */
    @Test
    public void testConfidenceCalculationAverageConditions() {
        double confidence = processor.calculateConfidence(
            2.5,    // 2.5 meters from tags
            2,      // 2 tags seen
            true,   // MegaTag 2
            1.0     // Moderate rotation (1 rad/s)
        );

        // Should have medium confidence (0.3 to 0.7)
        assertTrue(confidence >= 0.3 && confidence <= 0.7,
            "Average conditions should produce medium confidence");
    }

    /**
     * Test multi-camera pose fusion with high confidence.
     */
    @Test
    public void testMultiCameraFusionHighConfidence() {
        Pose2d[] poses = {
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(1.1, 1.05, Rotation2d.fromDegrees(2))
        };

        double[] confidences = {0.8, 0.7}; // Both above threshold

        Pose2d fusedPose = processor.fusePoses(poses, confidences);

        assertNotNull(fusedPose, "Fusion should succeed with high confidence poses");

        // Fused pose should be between the two poses
        assertTrue(fusedPose.getX() > 1.0 && fusedPose.getX() < 1.1,
            "Fused X should be between camera poses");
        assertTrue(fusedPose.getY() > 1.0 && fusedPose.getY() < 1.05,
            "Fused Y should be between camera poses");
    }

    /**
     * Test multi-camera pose fusion with low confidence.
     */
    @Test
    public void testMultiCameraFusionLowConfidence() {
        Pose2d[] poses = {
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(1.1, 1.05, Rotation2d.fromDegrees(2))
        };

        double[] confidences = {0.3, 0.2}; // Both below threshold

        Pose2d fusedPose = processor.fusePoses(poses, confidences);

        assertNull(fusedPose, "Fusion should fail with low confidence poses");
    }

    /**
     * Test outlier rejection with normal pose.
     */
    @Test
    public void testOutlierRejectionNormalPose() {
        Pose2d normalPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
        Pose2d meanPose = new Pose2d(1.05, 1.02, Rotation2d.fromDegrees(2));
        double stdDev = 0.1;

        boolean isOutlier = processor.isOutlier(normalPose, meanPose, stdDev);

        assertFalse(isOutlier, "Normal pose should not be rejected as outlier");
    }

    /**
     * Test outlier rejection with extreme pose.
     */
    @Test
    public void testOutlierRejectionExtremePose() {
        Pose2d extremePose = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(90));
        Pose2d meanPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
        double stdDev = 0.1;

        boolean isOutlier = processor.isOutlier(extremePose, meanPose, stdDev);

        assertTrue(isOutlier, "Extreme pose should be rejected as outlier");
    }

    /**
     * Test outlier rejection with rotation difference.
     */
    @Test
    public void testOutlierRejectionRotationDifference() {
        Pose2d rotatedPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90));
        Pose2d meanPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
        double stdDev = 0.1;

        boolean isOutlier = processor.isOutlier(rotatedPose, meanPose, stdDev);

        assertTrue(isOutlier, "Large rotation difference should be rejected");
    }

    /**
     * Test adaptive standard deviation calculation.
     */
    @Test
    public void testAdaptiveStandardDeviation() {
        double baseStdDev = 0.1;

        // Camera 0 is healthy (status 0)
        double adaptiveStdDev0 = processor.getAdaptiveStdDev(0, baseStdDev);

        // Should be >= base std dev
        assertTrue(adaptiveStdDev0 >= baseStdDev,
            "Adaptive std dev should be >= base std dev");

        // With velocity, should be higher than base
        assertTrue(adaptiveStdDev0 > baseStdDev,
            "Adaptive std dev should increase with velocity");
    }

    /**
     * Test pose history management.
     */
    @Test
    public void testPoseHistoryManagement() {
        Pose2d testPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        double timestamp = 0.0;
        double confidence = 0.8;
        int cameraIndex = 0;

        // Add poses to history
        for (int i = 0; i < 15; i++) {
            processor.addPoseToHistory(testPose, timestamp + i * 0.02, confidence, cameraIndex);
        }

        // Calculate statistics (should not crash)
        double[] stats = processor.calculatePoseStatistics();
        assertNotNull(stats, "Statistics should not be null");
        assertEquals(4, stats.length, "Statistics should have 4 elements");
    }

    /**
     * Test pose statistics calculation.
     */
    @Test
    public void testPoseStatisticsCalculation() {
        // Add several similar poses
        for (int i = 0; i < 5; i++) {
            Pose2d variedPose = new Pose2d(
                1.0 + i * 0.01,
                1.0 + i * 0.01,
                Rotation2d.fromDegrees(i * 2)
            );
            processor.addPoseToHistory(variedPose, i * 0.02, 0.8, 0);
        }

        double[] stats = processor.calculatePoseStatistics();

        // Check that statistics are reasonable
        assertNotNull(stats[0], "Mean X should not be null");
        assertNotNull(stats[1], "Mean Y should not be null");
        assertNotNull(stats[2], "Mean rotation should not be null");
        assertTrue(stats[3] > 0, "Standard deviation should be positive");
    }

    /**
     * Test health monitoring statistics.
     */
    @Test
    public void testHealthMonitoringStats() {
        // Simulate various observations
        processor.updateHealthStats(true, 0.8, 0);  // Good observation camera 0
        processor.updateHealthStats(true, 0.6, 1);  // Good observation camera 1
        processor.updateHealthStats(false, 0.2, 0); // Rejected observation camera 0

        // Perform health check (should not crash)
        processor.performHealthCheck();

        // Reset stats (should not crash)
        processor.resetHealthStats();
    }

    /**
     * Test vision latency constant.
     */
    @Test
    public void testVisionLatencyConstant() {
        assertEquals(0.05, AdvancedVisionProcessor.DEFAULT_VISION_LATENCY_SECONDS,
            "Vision latency should be 50ms");
    }
}
