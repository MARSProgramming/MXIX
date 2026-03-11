package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SystemConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * A disconnect-robust subsystem for interfacing with a Limelight camera for vision-based pose estimation.
 **/
public class Limelight2 extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    // Vision filtering constants
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters
    private static final double MAX_POSE_DIFFERENCE = 0.5; // meters
    private static final double MAX_ROTATION_DIFFERENCE = Math.toRadians(30); // radians
    private static final double MIN_TAG_AREA = 0.1; // percent
    private static final double LATENCY_THRESHOLD = 0.1;
    
    // Standard deviation calculation constants (similar to Northstar)
    private static final double XY_STD_DEV_COEFFICIENT = 0.01;
    private static final double THETA_STD_DEV_COEFFICIENT = 0.01;

    public Limelight2(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        LimelightHelpers.SetIMUMode(name, 3);
        LimelightHelpers.SetIMUAssistAlpha(name, 0.001);
        LimelightHelpers.SetFiducialIDFiltersOverride(name, SystemConstants.Limelights.getValidTagIDs());
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        // Basic validity checks
        if (poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2
        final Pose2d combinedPose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );

        // --- REJECTION LOGIC (inspired by Northstar) ---

        // 1. Reject if robot pose is off the field
        if (combinedPose.getX() < -FIELD_BORDER_MARGIN
                || combinedPose.getX() > FieldConstants.fieldLength
                || combinedPose.getY() < -FIELD_BORDER_MARGIN
                || combinedPose.getY() > FieldConstants.fieldWidth) {
            return Optional.empty();
        }

        // 2. Reject if pose is too far from odometry estimate
        double poseDistance = currentRobotPose.getTranslation()
            .getDistance(combinedPose.getTranslation());
        if (poseDistance > MAX_POSE_DIFFERENCE) {
            return Optional.empty();
        }

        // 3. Reject if rotation difference is too large
        double rotationDifference = Math.abs(
            currentRobotPose.getRotation()
                .minus(combinedPose.getRotation())
                .getRadians()
        );
        if (rotationDifference > MAX_ROTATION_DIFFERENCE) {
            return Optional.empty();
        }

        // 4. Reject if target area is too small (far away or ambiguous)
        double targetArea = LimelightHelpers.getTA(name);
        if (targetArea < MIN_TAG_AREA) {
            return Optional.empty();
        }

        // 5. Reject if latency is too high (for single-tag detections)
        if (poseEstimate_MegaTag2.tagCount == 1) {
            double latency = LimelightHelpers.getLatency_Pipeline(name);
            if (latency > LATENCY_THRESHOLD) {
                return Optional.empty();
            }
        }

        // --- DYNAMIC STANDARD DEVIATIONS (Northstar-style) ---
        
        // Calculate average distance to tags
        double avgDistance = poseEstimate_MegaTag2.avgTagDist;
        int tagCount = poseEstimate_MegaTag2.tagCount;

        // Calculate standard deviations based on distance and tag count
        // More tags and closer distance = lower std dev (higher confidence)
        double xyStdDev = XY_STD_DEV_COEFFICIENT 
            * Math.pow(avgDistance, 1.2) 
            / Math.pow(tagCount, 2.0);
        
        // Use high theta std dev to avoid trusting vision rotation too much
        // (Northstar style: only trust rotation with multi-tag)
        double thetaStdDev = 25.0;

        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

        posePublisher.set(combinedPose);

        // Update the pose estimate with combined pose
        poseEstimate_MegaTag2.pose = combinedPose;
        
        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    @Override
    public void periodic() {  
        if (RobotState.isDisabled()) {
            LimelightHelpers.SetThrottle(name, 100);
        } else {
            LimelightHelpers.SetThrottle(name, 0);
        }
    }
}