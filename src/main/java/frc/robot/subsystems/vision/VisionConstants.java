// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-shooter";
  public static String camera1Name = "limelight-back";

  // Named constants for magic numbers (defined first to avoid forward references)

  // Standard deviation baseline constants
  private static final double LINEAR_STD_DEV_BASE = 0.35; // Base linear std dev in meters
  private static final double ANGULAR_STD_DEV_BASE = 0.36; // Base angular std dev in radians
  private static final double STD_DEV_SAFETY_MULTIPLIER = 2.0; // Safety multiplier for uncertainty

  // MegaTag 2 multipliers
  public static final double MEGATAG_2_LINEAR_MULTIPLIER = 0.2; // MegaTag 2 provides more stable linear estimates
  public static final double MEGATAG_2_NO_ROTATION_DATA = Double.POSITIVE_INFINITY; // MegaTag 2 has no rotation data

  // Tag trust multipliers based on field location
  public static final double HUB_TAG_TRUST = 1.0; // High trust for hub tags (center field)
  public static final double OUTPOST_TAG_TRUST = 3.5; // Medium trust for outpost/tower tags
  public static final double TRENCH_NEUTRAL_TRUST = 1.0; // High trust for neutral zone trench tags
  public static final double TRENCH_ALLIANCE_TRUST = 9.0; // Low trust for alliance zone trench tags (poor angle)

  // Limelight IMU configuration
  public static final int LIMELIGHT_IMU_MODE = 4; // IMU mode 4: Extended Kalman Filter with vision update
  public static final double IMU_ASSIST_ALPHA = 0.001; // Complementary filter coefficient (low = trust gyro more)

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = LINEAR_STD_DEV_BASE * STD_DEV_SAFETY_MULTIPLIER; // Meters
  public static double angularStdDevBaseline = ANGULAR_STD_DEV_BASE * STD_DEV_SAFETY_MULTIPLIER; // Radians

  // Basic filtering thresholds
  public static double maxAmbiguityMt1 = 0.1;
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 * 1, // Camera 0
        1.0 * 1.2 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = MEGATAG_2_LINEAR_MULTIPLIER; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = MEGATAG_2_NO_ROTATION_DATA; // No rotation data available

  // Multipliers for dynamic standard deviation scaling
  public static double angularVelocityStdDevScale = 0.5; // Multiply variance by (1 + omega * scale)
  public static double zErrorStdDevScale = 2.0; // Multiply variance by (1 + zError * scale)

  // Tag trust multipliers based on tag ID
  public static double getTagStdevMultiplier(int tag) {
    switch (tag) {
      case 9, 10, 11, 2, 8, 5, 4, 3, 19, 20, 21, 24, 18, 27, 26, 25: // HUB TAGS
        return HUB_TAG_TRUST;
      case 14, 13, 15, 16, 29, 30, 31, 32: // OUTPOST, TOWER TAGS
        return OUTPOST_TAG_TRUST;
      case 1, 6, 22, 17: // TRENCH TAGS SEEN FROM NEUTRAL ZONE
        return TRENCH_NEUTRAL_TRUST;
      case 12, 7, 28, 23:
        return TRENCH_ALLIANCE_TRUST; // TRENCH TAGS SEEN FROM ALLIANCE ZONE
      default:
        return Double.POSITIVE_INFINITY; // Unknown tag, reject
    }
  }
}
