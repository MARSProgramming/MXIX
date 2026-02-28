package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.constants.SystemConstants.Limelights;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * A disconnect-robust subsystem for interfacing with a Limelight camera for vision-based pose estimation.
 **/

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static double CAMERA_STATIC_TRUST = 1.0;


    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
                   poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
                || poseEstimate_MegaTag2.pose.getX() < 0.0
                || poseEstimate_MegaTag2.pose.getX() > aprilTagLayout.getFieldLength()
                || poseEstimate_MegaTag2.pose.getY() < 0.0
                || poseEstimate_MegaTag2.pose.getY() > aprilTagLayout.getFieldWidth()
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag1.pose = new Pose2d(
            poseEstimate_MegaTag1.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        


       // final Matrix<N3, N1> standardDeviations = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
       final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.7, 0.7, 25);

        posePublisher.set(poseEstimate_MegaTag1.pose);
        return Optional.of(new Measurement(poseEstimate_MegaTag1, standardDeviations));
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

        LimelightHelpers.SetIMUMode(name, 3);
        LimelightHelpers.SetIMUAssistAlpha(name, 0.001);
        LimelightHelpers.SetFiducialIDFiltersOverride(name, Limelights.getValidTagIDs());
    }
}