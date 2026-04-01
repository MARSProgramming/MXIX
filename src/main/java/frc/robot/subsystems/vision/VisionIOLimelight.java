// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import dev.doglog.DogLog;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import frc.robot.util.LimelightHelpers;

/**
 * IO implementation for real Limelight hardware.
 * Handles NetworkTable data fetching, telemetry updates, and pose observation parsing.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;
  // Use DoubleArrayPublisher from NT for direct array writing (if we choose to)
  // but we will primarily use LimelightHelpers.SetRobotOrientation.


  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final List<Integer> tempTagIds = new ArrayList<>();
  private final List<PoseObservation> tempPoseObservations = new ArrayList<>();

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation from the drivetrain (used for MegaTag 2).
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  /**
   * Updates the VisionIO inputs object with the latest data from the Limelight NetworkTables.
   * This includes connection status, target observations, and any available pose observations
   * (Standard/MegaTag1 and MegaTag2).
   *
   * @param inputs The VisionIOInputs object to populate with fresh data.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    DogLog.log("Vision/Camera/" + name + "/Connected", inputs.connected);
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // --- CRITICAL MEGATAG 2 UPDATE ---
    // Feed the drivetrain IMU yaw into the Limelight.
    // This is required for MegaTag 2 to resolve ambiguity with a single tag.
    // The Limelight 4's internal gyro can drift independently of the swerve modules.
    double currentYawDegrees = rotationSupplier.get().getDegrees();

    // We update via LimelightHelpers to ensure the exact format Limelight expects.
    // Assuming 0 for yawRate, pitch, pitchRate, roll, rollRate since typical 2D odometry handles yaw primarily.
    LimelightHelpers.SetRobotOrientation(name, currentYawDegrees, 0, 0, 0, 0, 0);

    // Read new pose observations from NetworkTables
    tempTagIds.clear();
    tempPoseObservations.clear();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        int id = (int) rawSample.value[i];
        if (!tempTagIds.contains(id)) tempTagIds.add(id);
      }
      tempPoseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        int id = (int) rawSample.value[i];
        if (!tempTagIds.contains(id)) tempTagIds.add(id);
      }
      tempPoseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated by MegaTag2/IMU
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    if (inputs.poseObservations.length != tempPoseObservations.size()) {
      inputs.poseObservations = new PoseObservation[tempPoseObservations.size()];
    }
    for (int i = 0; i < tempPoseObservations.size(); i++) {
      inputs.poseObservations[i] = tempPoseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    if (inputs.tagIds.length != tempTagIds.size()) {
      inputs.tagIds = new int[tempTagIds.size()];
    }
    int iter = 0;
    for (int id : tempTagIds) {
      inputs.tagIds[iter++] = id;
    }
  }

  /**
   * Parses the 3D pose from a Limelight botpose double array.
   *
   * @param rawLLArray The raw double array containing translation and rotation data from NetworkTables.
   * @return A constructed Pose3d representing the robot's pose on the field.
   */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
