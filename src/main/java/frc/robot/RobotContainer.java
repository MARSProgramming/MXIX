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
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Maximum speed of the robot in meters per second, derived from TunerConstants.
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // The driver's controller.
    private final CommandXboxController pilot = new CommandXboxController(0);

    // Subsystems
    Swerve swervebase = new Swerve();
    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);
    private final Limelight shooterLimelight = new Limelight("llshooter");
    private final Limelight backLimelight = new Limelight("llback");

    // Autonomous routines manager
    private final AutoRoutines autoRoutines = new AutoRoutines(swervebase, shooterLimelight, backLimelight);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
        autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        // Default drive command: Field-centric driving with joystick inputs.
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
                swervebase,
                () -> -pilot.getLeftY(),
                () -> -pilot.getLeftX(),
                () -> -pilot.getRightX());

        // Aim and Drive command: Drives while automatically aiming at a target.
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
                swervebase,
                () -> -pilot.getLeftY(),
                () -> -pilot.getLeftX());

        // Left Bumper: While held, auto-aim at the speaker/target.
        pilot.leftBumper().whileTrue(aimAndDriveCommand);

        // Right Bumper: Lock heading to the nearest "diamond" (cardinal/intercardinal) direction.
        pilot.rightBumper().onTrue(Commands.runOnce(
                () -> manualDriveCommand.setLockedHeading(
                        FieldConstants.Orientations.getClosestDiamond(swervebase.getState().Pose))));

        // Set default commands
        swervebase.setDefaultCommand(manualDriveCommand);
        shooterLimelight.setDefaultCommand(updateShooterVision());
        backLimelight.setDefaultCommand(updateBackVision());

        // Back Button: Reset field-centric heading (gyro).
        pilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    /**
     * Creates a command to continuously update the robot's pose using the shooter Limelight.
     *
     * @return A command that runs in the background (default command).
     */
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

    /**
     * Creates a command to continuously update the robot's pose using the back Limelight.
     *
     * @return A command that runs in the background (default command).
     */
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
