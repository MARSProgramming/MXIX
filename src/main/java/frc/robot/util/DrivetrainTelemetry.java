package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class DrivetrainTelemetry extends SubsystemBase {
    private Swerve dt;
    StructPublisher<Pose2d> publisher;

    public DrivetrainTelemetry(Swerve drivetrain)  {
        dt = drivetrain;
         publisher = NetworkTableInstance.getDefault().getStructTopic("AdvantageKitPose", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
       publisher.set(dt.getState().Pose);      
    }
}