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


  // Basic filtering thresholds
  public static double maxAmbiguityMt1 = 0.1;
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.35 * 2; // Meters
  public static double angularStdDevBaseline = 0.36 * 2; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 * 1, // Camera 0
        1.0 * 1.2 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.2; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = 9999999; // No rotation data available

  // Multipliers for dynamic standard deviation scaling
  public static double angularVelocityStdDevScale = 0.5; // Multiply variance by (1 + omega * scale)
  public static double zErrorStdDevScale = 2.0; // Multiply variance by (1 + zError * scale)

  // variant trust based on tag ID.

    public static double getTagStdevMultiplier(int tag) {
    switch (tag) {
      case 9, 10, 11, 2, 8, 5, 4, 3, 19, 20, 21, 24, 18, 27, 26, 25: // HUB TAGS
        return 1.0;
      case 14, 13, 15, 16, 29, 30, 31, 32: // OUTPOST, TOWER TAGS
        return 3.5;
      case 1, 6, 22, 17: // TRENCH TAGS SEEN FROM NEUTRAL ZONE
        return 1.0;
      case 12, 7, 28, 23:
        return 9.0; // TRENCH TAGS SEEN FROM ALLIANCE ZONE
      default:
        return Double.POSITIVE_INFINITY; // Unknown tag, reject
    }
  }

}