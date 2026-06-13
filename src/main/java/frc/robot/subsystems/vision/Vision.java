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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> rotationSupplier;
  private final VisionIO[] io;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  

    // gives us leeway to correct in auto
    private static final double MAX_TAG_DIST = 6.5; // reject poses further away than 10 meters. (Impossible)
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters

  public Vision(
      VisionConsumer consumer,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.rotationSupplier = rotationSupplier;
    this.speedsSupplier = speedsSupplier;
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
    
    if (RobotState.isDisabled()) {
            LimelightHelpers.SetThrottle(VisionConstants.camera0Name, 100);
            LimelightHelpers.SetThrottle(VisionConstants.camera1Name, 100);
        } else {
            LimelightHelpers.SetThrottle(VisionConstants.camera0Name, 0);
            LimelightHelpers.SetThrottle(VisionConstants.camera1Name, 0);
        }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
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

      // Group observations by timestamp to process frames individually
      // (Since we have MT1 and MT2 for the same frames, they will have matching timestamps)
      List<PoseObservation> frameObservations = new ArrayList<>();
      
      Set<Double> timestamps = new HashSet<>();
      for (var obs : inputs[cameraIndex].poseObservations) {
        timestamps.add(obs.timestamp());
      }

      for (double timestamp : timestamps) {
        PoseObservation mt1 = null;
        PoseObservation mt2 = null;
        for (var obs : inputs[cameraIndex].poseObservations) {
          if (Math.abs(obs.timestamp() - timestamp) < 1e-4) {
            if (obs.type() == PoseObservationType.MEGATAG_1) {
              mt1 = obs;
            } else if (obs.type() == PoseObservationType.MEGATAG_2) {
              mt2 = obs;
            }
          }
        }

        if (RobotState.isDisabled()) {
          // When disabled, we ONLY use MegaTag 1 to initialize/correct heading and position
          if (mt1 != null) {
            frameObservations.add(mt1);
          }
        } else {
          // When enabled:
          if (mt1 != null && mt2 != null) {
            // Compare camera-solved yaw (MT1) with our current estimated yaw
            double currentYaw = rotationSupplier.get().getRadians();
            double visionYaw = mt1.pose().getRotation().toRotation2d().getRadians();
            double yawError = Math.abs(MathUtil.angleModulus(visionYaw - currentYaw));

            // Check if the robot is stationary (speeds are low)
            ChassisSpeeds speeds = speedsSupplier.get();
            boolean isStationary = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < VisionConstants.maxSpeedToCorrectYaw
                && Math.abs(speeds.omegaRadiansPerSecond) < VisionConstants.maxOmegaToCorrectYaw;

            // Check if the observation is high confidence and close range
            boolean isHighConfidence = mt1.averageTagDistance() < VisionConstants.maxDistanceToCorrectYaw
                && (mt1.tagCount() > 1 || mt1.ambiguity() < maxAmbiguityMt1);

            if (yawError > Units.degreesToRadians(maxYawErrorToUseMegatag2) && isStationary && isHighConfidence) {
              // Large yaw error while stationary and seeing a clear close-range tag: 
              // use MegaTag 1 to correct the gyro/pose yaw
              frameObservations.add(mt1);
            } else {
              // Yaw is already aligned, or robot is moving, or tag is too far/low-confidence:
              // use MegaTag 2 for stable translation tracking
              frameObservations.add(mt2);
            }
          } else if (mt2 != null) {
            frameObservations.add(mt2);
          } else if (mt1 != null) {
            frameObservations.add(mt1);
          }
        }
      }

      // Loop over pose observations
      for (var observation : frameObservations) {

    // Check whether to reject pose
       boolean rejectPose =
        observation.tagCount() == 0 // Must have at least one tag
            || (observation.type() == PoseObservationType.MEGATAG_1 && observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguityMt1) // Reject all single tag MT1 (EXTREMELY Low Confidence)
            || (observation.type() == PoseObservationType.MEGATAG_2 && observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) // Single tag MT2 still needs ambiguity check
        || Math.abs(observation.pose().getZ())
            > maxZError // Must have realistic Z coordinate
        // Must be within the field boundaries
        || !(Double.isFinite(observation.pose().getX()))
        || !(Double.isFinite(observation.pose().getY()))
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN
        || observation.averageTagDistance() > MAX_TAG_DIST;

        robotPoses.add(observation.pose());

        // Reject if rotating too fast (only apply to MegaTag 1)
        boolean omegaRejected = (observation.type() == PoseObservationType.MEGATAG_1) 
            && Math.abs(speedsSupplier.get().omegaRadiansPerSecond) > 2.0;

        if (rejectPose || omegaRejected) {
        robotPosesRejected.add(observation.pose());
        continue;
        } else {
        robotPosesAccepted.add(observation.pose());
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 1.8) / observation.tagCount() * tagStdevMultiplier;
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

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}