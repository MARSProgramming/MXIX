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
import frc.robot.commands.FeedCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PrepareSupershot;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;
import frc.robot.util.ShotSetup;

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
    private final CommandXboxController testPilot = new CommandXboxController(0);
    private final CommandXboxController drivePilot = new CommandXboxController(1);

    // Subsystems
    Cowl mCowl = new Cowl();
    Flywheel mFlywheel = new Flywheel();
    Feeder mFeeder = new Feeder();
    Floor mFloor = new Floor();
    IntakePivot mIntakePivot = new IntakePivot();
    ShotSetup shotSetup = new ShotSetup();
    Swerve swerve = new Swerve();
    IntakeRollers mIntakeRollers = new IntakeRollers();

    Command prepShotCommand = new PrepareSupershot(
      shotSetup, 
      swerve, 
      mFlywheel, 
      mCowl, 
      () -> -drivePilot.getLeftY(), 
      () -> -drivePilot.getLeftX());

  //  Superstructure mSuperstructure = new Superstructure(mCowl, mFlywheel, mFeeder, mFloor);

   // DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);

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
      testPilot.leftTrigger().whileTrue(mFeeder.setPercentOutTunable());
      testPilot.rightTrigger().whileTrue(mFlywheel.setPercentOutTunable());
       testPilot.y().whileTrue(mIntakeRollers.setTunable());
       testPilot.b().whileTrue(mFloor.setPercentOutTunable());


    //  testPilot.povUp().whileTrue(mCowl.setPositionTunable()); // Min 0 Max 1.8
    //  testPilot.povDown().onTrue(mCowl.home());
     testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
     testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());



      final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
          swerve,
          () -> -drivePilot.getLeftY(),
          () -> -drivePilot.getLeftX(),
          () -> -drivePilot.getRightX());


      swerve.setDefaultCommand(manualDriveCommand);
      drivePilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));



    }
}
