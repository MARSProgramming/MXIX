// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    //ivate final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController pilot = new CommandXboxController(0);
    Swerve swervebase = new Swerve();
    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);
    private final Limelight shooterLimelight = new Limelight("llshooter");
    private final Limelight backLimelight = new Limelight("llback");

    private final AutoRoutines autoRoutines = new AutoRoutines(swervebase, shooterLimelight, backLimelight);


    public RobotContainer() {
        configureBindings();
        autoRoutines.configure(); // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
    }

    private void configureBindings() {  

        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swervebase, 
            () -> -pilot.getLeftY(), 
            () -> -pilot.getLeftX(), 
            () -> -pilot.getRightX()
        );

        swervebase.setDefaultCommand(manualDriveCommand); // Handles teleoperated driving
        shooterLimelight.setDefaultCommand(updateShooterVision());
        backLimelight.setDefaultCommand(updateBackVision());

        pilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())); // Re-seeds field-centric heading when 'back' button is pressed

        // Locks the heading to our nearest "diamond orientation."
        // Disables upon first rotational input from joystick.
        pilot.leftBumper().onTrue(Commands.runOnce(
          () -> manualDriveCommand.setLockedHeading(
            FieldConstants.Orientations.getClosestDiamond(swervebase.getState().Pose))
            )
        ); 
    }

    // Update the robot's pose estimate using vision measurements from a Limelight
    private Command updateShooterVision() {
        return shooterLimelight.run(() -> {
            final Pose2d currentRobotPose = swervebase.getState().Pose;
            final Optional<Limelight.Measurement> measurement = shooterLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swervebase.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }

    
    // Update the robot's pose estimate using vision measurements from a Limelight
    private Command updateBackVision() {
        return backLimelight.run(() -> {
            final Pose2d currentRobotPose = swervebase.getState().Pose;
            final Optional<Limelight.Measurement> measurement = backLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swervebase.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}
