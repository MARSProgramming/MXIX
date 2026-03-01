package frc.robot.subsystems;


import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class DrivetrainTelemetry extends SubsystemBase {
    private Swerve dt;

    public DrivetrainTelemetry(Swerve drivetrain)  {
        dt = drivetrain;
    }

    @Override
    public void periodic() {       
       DogLog.log("DogLogSwerve/EstimatedRobotPose", dt.getState().Pose);
    }
}