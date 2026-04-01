package frc.robot.subsystems.vision;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for VisionConstants to ensure tag trust multipliers are correctly configured.
 *
 * <p>These tests verify that the tag trust system is working as expected:
 * <ul>
 *   <li>Hub tags should have high trust (low multiplier)</li>
 *   <li>Outpost tags should have medium trust</li>
 *   <li>Neutral zone trench tags should have high trust</li>
 *   <li>Alliance zone trench tags should have low trust (high multiplier)</li>
 *   <li>Unknown tags should be rejected (infinite multiplier)</li>
 * </ul>
 */
public class VisionConstantsTest {

    /**
     * Test that hub tags return the expected high trust multiplier.
     * Hub tags are located at the center of the field and provide reliable pose estimates.
     */
    @Test
    public void testHubTagTrustMultiplier() {
        // Known hub tag IDs from 2026 field layout
        int[] hubTags = {9, 10, 11, 2, 8, 5, 4, 3, 19, 20, 21, 24, 18, 27, 26, 25};

        for (int tagId : hubTags) {
            double multiplier = VisionConstants.getTagStdevMultiplier(tagId);
            assertEquals(VisionConstants.HUB_TAG_TRUST, multiplier,
                "Hub tag " + tagId + " should have high trust multiplier");
            assertEquals(1.0, multiplier,
                "Hub tag multiplier should be 1.0 (highest trust)");
        }
    }

    /**
     * Test that outpost/tower tags return the expected medium trust multiplier.
     * Outpost tags are located at the corners of the field and have moderate reliability.
     */
    @Test
    public void testOutpostTagTrustMultiplier() {
        // Known outpost/tower tag IDs from 2026 field layout
        int[] outpostTags = {14, 13, 15, 16, 29, 30, 31, 32};

        for (int tagId : outpostTags) {
            double multiplier = VisionConstants.getTagStdevMultiplier(tagId);
            assertEquals(VisionConstants.OUTPOST_TAG_TRUST, multiplier,
                "Outpost tag " + tagId + " should have medium trust multiplier");
            assertEquals(3.5, multiplier,
                "Outpost tag multiplier should be 3.5 (medium trust)");
        }
    }

    /**
     * Test that neutral zone trench tags return the expected high trust multiplier.
     * These tags are viewed from the neutral zone and provide good angles.
     */
    @Test
    public void testNeutralZoneTrenchTagTrustMultiplier() {
        // Known neutral zone trench tag IDs
        int[] neutralZoneTags = {1, 6, 22, 17};

        for (int tagId : neutralZoneTags) {
            double multiplier = VisionConstants.getTagStdevMultiplier(tagId);
            assertEquals(VisionConstants.TRENCH_NEUTRAL_TRUST, multiplier,
                "Neutral zone trench tag " + tagId + " should have high trust multiplier");
            assertEquals(1.0, multiplier,
                "Neutral zone trench tag multiplier should be 1.0 (high trust)");
        }
    }

    /**
     * Test that alliance zone trench tags return the expected low trust multiplier.
     * These tags are viewed from a poor angle (alliance zone) and have low reliability.
     */
    @Test
    public void testAllianceZoneTrenchTagTrustMultiplier() {
        // Known alliance zone trench tag IDs
        int[] allianceZoneTags = {12, 7, 28, 23};

        for (int tagId : allianceZoneTags) {
            double multiplier = VisionConstants.getTagStdevMultiplier(tagId);
            assertEquals(VisionConstants.TRENCH_ALLIANCE_TRUST, multiplier,
                "Alliance zone trench tag " + tagId + " should have low trust multiplier");
            assertEquals(9.0, multiplier,
                "Alliance zone trench tag multiplier should be 9.0 (low trust)");
        }
    }

    /**
     * Test that unknown tags are rejected with infinite multiplier.
     * This prevents the vision system from using invalid tag IDs.
     */
    @Test
    public void testUnknownTagRejected() {
        // Tag IDs that don't exist on the field
        int[] unknownTags = {0, 100, 255, -1, 999};

        for (int tagId : unknownTags) {
            double multiplier = VisionConstants.getTagStdevMultiplier(tagId);
            assertEquals(Double.POSITIVE_INFINITY, multiplier,
                "Unknown tag " + tagId + " should be rejected with infinite multiplier");
        }
    }

    /**
     * Test that MegaTag 2 multipliers are configured correctly.
     * MegaTag 2 provides more stable linear estimates but no rotation data.
     */
    @Test
    public void testMegatag2Multipliers() {
        // Linear multiplier should be low (more stable)
        assertEquals(0.2, VisionConstants.linearStdDevMegatag2Factor,
            "MegaTag 2 linear multiplier should be 0.2");

        // Angular multiplier should be infinite (no rotation data)
        assertEquals(Double.POSITIVE_INFINITY, VisionConstants.angularStdDevMegatag2Factor,
            "MegaTag 2 angular multiplier should be infinite (no rotation data)");
    }

    /**
     * Test that standard deviation baselines are correctly calculated.
     * The baselines should use the safety multiplier for uncertainty.
     */
    @Test
    public void testStandardDeviationBaselines() {
        // Expected values: base * safety multiplier (2.0)
        double expectedLinear = 0.35 * 2.0; // 0.7
        double expectedAngular = 0.36 * 2.0; // 0.72

        assertEquals(expectedLinear, VisionConstants.linearStdDevBaseline, 0.001,
            "Linear std dev baseline should be base * safety multiplier");
        assertEquals(expectedAngular, VisionConstants.angularStdDevBaseline, 0.001,
            "Angular std dev baseline should be base * safety multiplier");
    }

    /**
     * Test that dynamic standard deviation scaling factors are positive.
     * These factors should never be negative or zero.
     */
    @Test
    public void testDynamicScalingFactorsArePositive() {
        assertTrue(VisionConstants.angularVelocityStdDevScale > 0,
            "Angular velocity scale factor should be positive");
        assertTrue(VisionConstants.zErrorStdDevScale > 0,
            "Z error scale factor should be positive");
    }

    /**
     * Test that IMU configuration values are reasonable.
     * IMU mode and filter coefficient should be within expected ranges.
     */
    @Test
    public void testImuConfiguration() {
        // IMU mode should be a valid Limelight mode (typically 0-10)
        assertTrue(VisionConstants.LIMELIGHT_IMU_MODE >= 0 && VisionConstants.LIMELIGHT_IMU_MODE <= 10,
            "IMU mode should be within valid range (0-10)");

        // Filter coefficient should be between 0 and 1
        assertTrue(VisionConstants.IMU_ASSIST_ALPHA > 0 && VisionConstants.IMU_ASSIST_ALPHA < 1,
            "IMU assist alpha should be between 0 and 1");

        // Alpha should be small (trust gyro more than vision)
        assertTrue(VisionConstants.IMU_ASSIST_ALPHA < 0.1,
            "IMU assist alpha should be small (< 0.1) to trust gyro more");
    }

    /**
     * Test that camera names are configured and not empty.
     * Camera names must match Limelight web UI configuration.
     */
    @Test
    public void testCameraNamesConfigured() {
        assertNotNull(VisionConstants.camera0Name,
            "Camera 0 name should be configured");
        assertNotNull(VisionConstants.camera1Name,
            "Camera 1 name should be configured");
        assertFalse(VisionConstants.camera0Name.isEmpty(),
            "Camera 0 name should not be empty");
        assertFalse(VisionConstants.camera1Name.isEmpty(),
            "Camera 1 name should not be empty");

        // Verify camera names match expected Limelight naming convention
        assertTrue(VisionConstants.camera0Name.toLowerCase().contains("limelight"),
            "Camera 0 name should contain 'limelight'");
        assertTrue(VisionConstants.camera1Name.toLowerCase().contains("limelight"),
            "Camera 1 name should contain 'limelight'");
    }

    /**
     * Test that basic filtering thresholds are reasonable.
     * These thresholds prevent invalid pose observations from being used.
     */
    @Test
    public void testFilteringThresholds() {
        // Z error threshold should be positive and reasonable (meters)
        assertTrue(VisionConstants.maxZError > 0 && VisionConstants.maxZError < 2.0,
            "Z error threshold should be positive and less than 2 meters");
    }
}
