package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AdvancedVisionProcessorTest {

    private AdvancedVisionProcessor processor;

    @BeforeEach
    public void setup() {
        processor = new AdvancedVisionProcessor(0.0, 0.0, 0.0);
    }

    @Test
    public void testGetConfidenceScore_Megatag2Bonus() {
        double mt1Confidence = processor.calculateConfidence(0.5, 2, false, 0.0);
        double mt2Confidence = processor.calculateConfidence(0.5, 2, true, 0.0);

        assertTrue(mt2Confidence > mt1Confidence, "MegaTag2 should have higher confidence score under identical conditions");
    }

    @Test
    public void testOutlierDetection() {
        Pose2d meanPose = new Pose2d(5.0, 5.0, new Rotation2d(0));
        Pose2d validPose = new Pose2d(5.1, 5.1, new Rotation2d(0.01));
        Pose2d outlierPose = new Pose2d(15.0, 15.0, new Rotation2d(Math.PI));

        double stdDev = 0.5;

        assertFalse(processor.isOutlier(validPose, meanPose, stdDev), "Valid pose should not be rejected");
        assertTrue(processor.isOutlier(outlierPose, meanPose, stdDev), "Pose far away should be rejected");
    }

    @Test
    public void testFusePoses() {
        Pose2d pose1 = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        Pose2d pose2 = new Pose2d(3.0, 3.0, Rotation2d.kZero);

        Pose2d[] poses = {pose1, pose2};
        double[] confidences = {1.0, 1.0}; // Equal weight

        Pose2d fused = processor.fusePoses(poses, confidences);
        assertNotNull(fused);
        assertEquals(2.0, fused.getX(), 0.01);
        assertEquals(2.0, fused.getY(), 0.01);
    }
}
