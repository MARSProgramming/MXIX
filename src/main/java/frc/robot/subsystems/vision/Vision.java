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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final DoubleSupplier omegaSupplier;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean wasDisabled = false;
  

    // gives us leeway to correct in auto
    private static final double MAX_TAG_DIST = 6.5; // reject poses further away than 10 meters. (Impossible)
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters

  public Vision(VisionConsumer consumer, DoubleSupplier omegaSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.omegaSupplier = omegaSupplier;
    this.io = io;

    LimelightHelpers.SetIMUMode(camera0Name, 4);
    LimelightHelpers.SetIMUMode(camera1Name, 4);
    LimelightHelpers.SetIMUAssistAlpha(camera0Name, 0.001);
    LimelightHelpers.SetIMUAssistAlpha(camera1Name, 0.001);

    // Initialize inputs
    this.inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    
    boolean isDisabled = RobotState.isDisabled();
    if (isDisabled != wasDisabled) {
        int throttle = isDisabled ? 100 : 0;
        LimelightHelpers.SetThrottle(VisionConstants.camera0Name, throttle);
        LimelightHelpers.SetThrottle(VisionConstants.camera1Name, throttle);
        wasDisabled = isDisabled;
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();
      List<Double> tagStdevMultipliers = new ArrayList<>();

      // Add tag poses
      double tagStdevMultiplier = Double.POSITIVE_INFINITY;
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }

        double tagStdevMultiplierCandidate = getTagStdevMultiplier(tagId);
        if (tagStdevMultiplierCandidate < tagStdevMultiplier) {
          tagStdevMultiplier = tagStdevMultiplierCandidate;
        }
      }
      
      tagStdevMultipliers.add(tagStdevMultiplier);

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {

    // Check whether to reject pose
       boolean rejectPose =
        observation.tagCount() == 0 // Must have at least one tag
            || (observation.type() == PoseObservationType.MEGATAG_1 && observation.tagCount() == 1) // Absolutely reject all 1-tag MT1 poses (Trust MT2 instead)
        || Math.abs(observation.pose().getZ())
            > maxZError // Must have realistic Z coordinate
        // Must be within the field boundaries
        || !(Double.isFinite(observation.pose().getX()))
        || !(Double.isFinite(observation.pose().getY()))
        || observation.pose().getX() < -FIELD_BORDER_MARGIN
        || observation.pose().getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
        || observation.pose().getY() < -FIELD_BORDER_MARGIN
        || observation.pose().getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN
        || observation.averageTagDistance() > MAX_TAG_DIST;

        robotPoses.add(observation.pose());

        if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
        } else {
        robotPosesAccepted.add(observation.pose());
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 1.8) / observation.tagCount() * tagStdevMultiplier;
        
        // Add penalty for high angular velocity (blur/gyro skew)
        stdDevFactor *= (1.0 + (Math.abs(omegaSupplier.getAsDouble()) * angularVelocityStdDevScale));
        
        // Add penalty for height displacement
        stdDevFactor *= (1.0 + (Math.abs(observation.pose().getZ()) * zErrorStdDevScale));

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    DogLog.log(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    DogLog.log(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    DogLog.log(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    DogLog.log(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /**
   * Calculates the Limelight offset assuming Limelight web UI offset is 0.
   * Prints the calculated Transform3d to standard output and DogLog.
   * 
   * @param cameraIndex The index of the camera.
   * @param knownRobotPose The exact physical pose of the robot.
   */
  public void calculateAndLogLimelightOffset(int cameraIndex, Pose3d knownRobotPose) {
    if (cameraIndex < 0 || cameraIndex >= inputs.length || inputs[cameraIndex].poseObservations.length == 0) {
      System.out.println("Cannot calculate Limelight offset: Camera " + cameraIndex + " not connected or no pose seen.");
      return;
    }
    
    // Grab the latest pose from the Limelight (botpose with 0 offset = camera pose)
    Pose3d zeroOffsetBotPose = inputs[cameraIndex].poseObservations[0].pose();
    Transform3d offset = new Transform3d(knownRobotPose, zeroOffsetBotPose);
    
    System.out.println("=== Limelight " + cameraIndex + " Offset Calculation ===");
    System.out.println("X Offset (m): " + offset.getX());
    System.out.println("Y Offset (m): " + offset.getY());
    System.out.println("Z Offset (m): " + offset.getZ());
    System.out.println("Roll Offset (deg): " + Math.toDegrees(offset.getRotation().getX()));
    System.out.println("Pitch Offset (deg): " + Math.toDegrees(offset.getRotation().getY()));
    System.out.println("Yaw Offset (deg): " + Math.toDegrees(offset.getRotation().getZ()));
    System.out.println("=============================================");
    
    DogLog.log("Vision/Camera" + cameraIndex + "/CalculatedOffset/X", offset.getX());
    DogLog.log("Vision/Camera" + cameraIndex + "/CalculatedOffset/Y", offset.getY());
    DogLog.log("Vision/Camera" + cameraIndex + "/CalculatedOffset/Z", offset.getZ());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}