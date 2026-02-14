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
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
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
    Cowl mCowl = new Cowl();
    Flywheel mFlywheel = new Flywheel();
    Feeder mFeeder = new Feeder();

    Superstructure mSuperstructure = new Superstructure(mCowl, mFlywheel, mFeeder);

    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);

    // Autonomous routines manager
   // private final AutoRoutines autoRoutines = new AutoRoutines(swervebase, shooterLimelight, backLimelight);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
      //  autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        pilot.rightTrigger().whileTrue(mSuperstructure.feedAndRunFlywheel());

        pilot.a().whileTrue(mFeeder.setVelocityTunable());
        pilot.b().whileTrue(mFlywheel.setVelocityTunable());
    }
}
