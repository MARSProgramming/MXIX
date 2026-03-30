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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;

/**
 * Subsystem responsible for managing Limelight vision updates.
 *
 * <p>Vision Processing Pipeline:
 * <ol>
 *   <li>Read pose observations from Limelight cameras via NetworkTables</li>
 *   <li>Reject poses based on:
 *     <ul>
 *       <li>Tag count (must have at least 1 tag)</li>
 *       <li>Pose ambiguity (single-tag MegaTag 1 poses rejected)</li>
 *       <li>Z-coordinate errors (height displacement > maxZError)</li>
 *       <li>Field boundaries (poses outside field + margin rejected)</li>
 *       <li>Tag distance (poses too far from tags rejected)</li>
 *       <li>Field distance (poses too far from field center rejected)</li>
 *     </ul>
 *   </li>
 *   <li>Calculate dynamic standard deviations based on:
 *     <ul>
 *       <li>Tag distance (power law: distance^TAG_DISTANCE_EXPONENT / tagCount)</li>
 *       <li>Angular velocity (fast rotation causes blur - penalty applied)</li>
 *       <li>Z-coordinate error (height displacement - penalty applied)</li>
 *       <li>Tag ID (different field locations have different trust levels)</li>
 *       <li>Camera index (cameras may have different calibration quality)</li>
 *       <li>MegaTag 2 vs MegaTag 1 (MegaTag 2 gets lower linear std dev)</li>
 *     </ul>
 *   </li>
 *   <li>Feed accepted poses to Kalman filter for pose estimation</li>
 * </ol>
 *
 * <p>Standard Deviation Formula:
 * <pre>
 * stdDevFactor = (tagDistance ^ TAG_DISTANCE_EXPONENT) / tagCount * tagStdevMultiplier
 * stdDevFactor *= (1.0 + |angularVelocity| * ANGULAR_VELOCITY_SCALE)
 * stdDevFactor *= (1.0 + |zError| * Z_ERROR_SCALE)
 * linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor * MEGATAG_2_FACTOR (if applicable)
 * angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor * MEGATAG_2_FACTOR (if applicable)
 * </pre>
 *
 * <p>Error Handling:
 * <ul>
 *   <li>Camera disconnection alerts displayed on Driver Station</li>
 *   <li>Staleness detection switches to odometry-only if no vision received for timeout period</li>
 *   <li>Invalid poses rejected before reaching pose estimator</li>
 *   <li>NetworkTables errors caught and logged without crashing robot code</li>
 * </ul>
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final DoubleSupplier omegaSupplier;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean wasDisabled = false;

  // Staleness detection
  private double lastValidVisionTimestamp = 0;
  private static final double VISION_STALENESS_TIMEOUT_SECONDS = 0.5;
  private final Alert visionStaleAlert =
      new Alert("Vision data stale - using odometry only. Check camera connections!", AlertType.kWarning);

  // Distance from field center to reject poses (meters) - prevents wildly incorrect poses
  private static final double MAX_FIELD_CENTER_DISTANCE = 5.0;
  
  // Pre-allocate lists to avoid garbage collection overhead in periodic loop
  private final List<Pose3d> allTagPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  
  // Reusable lists per camera to avoid GC loops
  private final List<Pose3d> tempTagPoses = new ArrayList<>();
  private final List<Pose3d> tempRobotPoses = new ArrayList<>();
  private final List<Pose3d> tempRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> tempRobotPosesRejected = new ArrayList<>();
  private final List<Double> tempTagStdevMultipliers = new ArrayList<>();

    // gives us leeway to correct in auto
    private static final double MAX_TAG_DIST = 6.5; // reject poses further away than 6.5 meters
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters - margin around field boundaries

    // Tag distance power law exponent - empirically derived for trust scaling
    private static final double TAG_DISTANCE_EXPONENT = 1.8;

  /**
   * Constructs the Vision subsystem.
   *
   * @param consumer Consumer to accept standard deviation tagged pose updates.
   * @param omegaSupplier Supplier for the robot's angular velocity in radians per second.
   * @param io Instances of VisionIO representing the physical cameras.
   */
  public Vision(VisionConsumer consumer, DoubleSupplier omegaSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.omegaSupplier = omegaSupplier;
    this.io = io;

    LimelightHelpers.SetIMUMode(camera0Name, LIMELIGHT_IMU_MODE);
    LimelightHelpers.SetIMUMode(camera1Name, LIMELIGHT_IMU_MODE);
    LimelightHelpers.SetIMUAssistAlpha(camera0Name, IMU_ASSIST_ALPHA);
    LimelightHelpers.SetIMUAssistAlpha(camera1Name, IMU_ASSIST_ALPHA);

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
    try {
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

      // Check for vision data staleness
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean anyCameraConnected = false;
      for (int i = 0; i < io.length; i++) {
        if (inputs[i].connected) {
          anyCameraConnected = true;
          break;
        }
      }

      if (anyCameraConnected && (currentTime - lastValidVisionTimestamp) > VISION_STALENESS_TIMEOUT_SECONDS) {
        visionStaleAlert.set(true);
      } else {
        visionStaleAlert.set(false);
      }

      // Clear reusable lists
      allTagPoses.clear();
      allRobotPoses.clear();
      allRobotPosesAccepted.clear();
      allRobotPosesRejected.clear();

      // Loop over cameras
      for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
        // Update disconnected alert
        disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

        // Clear camera specific lists
        tempTagPoses.clear();
        tempRobotPoses.clear();
        tempRobotPosesAccepted.clear();
        tempRobotPosesRejected.clear();
        tempTagStdevMultipliers.clear();

        // Add tag poses
        double tagStdevMultiplier = Double.POSITIVE_INFINITY;
        for (int tagId : inputs[cameraIndex].tagIds) {
          var tagPose = aprilTagLayout.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tempTagPoses.add(tagPose.get());
          }

          double tagStdevMultiplierCandidate = getTagStdevMultiplier(tagId);
          if (tagStdevMultiplierCandidate < tagStdevMultiplier) {
            tagStdevMultiplier = tagStdevMultiplierCandidate;
          }
        }

        tempTagStdevMultipliers.add(tagStdevMultiplier);

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
          || observation.averageTagDistance() > MAX_TAG_DIST
          || observation.pose().getTranslation().getDistance(new Translation3d(
              FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, 0.0)) > MAX_FIELD_CENTER_DISTANCE; // Reject poses too far from field center

          tempRobotPoses.add(observation.pose());

          if (rejectPose) {
          tempRobotPosesRejected.add(observation.pose());
          continue;
          } else {
          tempRobotPosesAccepted.add(observation.pose());
          }

          // Calculate standard deviations using named constant
          double stdDevFactor = Math.pow(observation.averageTagDistance(), TAG_DISTANCE_EXPONENT) / observation.tagCount() * tagStdevMultiplier;

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

          // Send vision observation and update last valid timestamp
          lastValidVisionTimestamp = currentTime;
          consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tempTagPoses.toArray(new Pose3d[tempTagPoses.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          tempRobotPoses.toArray(new Pose3d[tempRobotPoses.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          tempRobotPosesAccepted.toArray(new Pose3d[tempRobotPosesAccepted.size()]));
      DogLog.log(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          tempRobotPosesRejected.toArray(new Pose3d[tempRobotPosesRejected.size()]));
      allTagPoses.addAll(tempTagPoses);
      allRobotPoses.addAll(tempRobotPoses);
      allRobotPosesAccepted.addAll(tempRobotPosesAccepted);
      allRobotPosesRejected.addAll(tempRobotPosesRejected);
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

    } catch (Exception e) {
      // Catch any exceptions in vision processing to prevent robot code crash
      edu.wpi.first.wpilibj.DriverStation.reportError("Vision processing error: " + e.getMessage(), e.getStackTrace());
      visionStaleAlert.set(true); // Switch to odometry-only mode on error
    }
  }

  /**
   * Calculates the Limelight offset assuming Limelight web UI offset is 0.
   * Prints the calculated Transform3d to standard output and DogLog.
   *
   * <p>Note: System.out.println is used here intentionally for calibration output
   * to ensure visibility during camera setup and tuning.
   *
   * @param cameraIndex The index of the camera.
   * @param knownRobotPose The exact physical pose of the robot.
   */
  @SuppressWarnings("resource") // NetworkTables resource access
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