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
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.Unjam;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Settings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
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

    private final Limelight shooterLimelight = new Limelight("limelight-shooter");
    private final Limelight backLimelight = new Limelight("limelight-back");


    private final CommandXboxController drivePilot = new CommandXboxController(0);
    private final CommandXboxController testPilot = new CommandXboxController(1);

    // Subsystems
    Cowl mCowl = new Cowl();
    Flywheel mFlywheel = new Flywheel();
    Feeder mFeeder = new Feeder();
    Floor mFloor = new Floor();
    IntakePivot mIntakePivot = new IntakePivot();
    ShotSetup shotSetup = new ShotSetup();
    Swerve swerve = new Swerve();
    IntakeRollers mIntakeRollers = new IntakeRollers();
    private DrivetrainTelemetry dtTelem = new DrivetrainTelemetry(swerve);

    FastClimber mFastClimber = new FastClimber();

    // Autonomous routines manager
    private final AutoRoutines autoRoutines = new AutoRoutines(swerve, mCowl, mFastClimber, mFeeder, mFloor, mFlywheel, mIntakePivot, mIntakeRollers, shooterLimelight);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        configureTestBindings();
        autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

        
      final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
          swerve,
          () -> -drivePilot.getLeftY(),
          () -> -drivePilot.getLeftX(),
          () -> -drivePilot.getRightX());
    
      // driving commands
      swerve.setDefaultCommand(manualDriveCommand);
      drivePilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));

      shooterLimelight.setDefaultCommand(updateShooterVision());
      backLimelight.setDefaultCommand(updateBackVision());

      drivePilot.leftTrigger().whileTrue(mIntakeRollers.setPercentOutCommand(Settings.IntakeSystemSettings.INTAKING_STANDARD_DUTYCYCLE));

      drivePilot.rightTrigger().whileTrue(new AimAndShoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers));
      drivePilot.leftBumper().whileTrue(new Unjam(mFeeder, mFloor, mIntakeRollers));

      drivePilot.povLeft().whileTrue(mIntakePivot.retractCommand());
      drivePilot.povRight().whileTrue(mIntakePivot.deployCommand());

      drivePilot.povDown().onTrue(mCowl.home());

    }

    private void configureTestBindings() {
        // shooting
        testPilot.rightTrigger().whileTrue(mFlywheel.setVelocityTunable());
        testPilot.leftTrigger().whileTrue(
        mFloor.setPercentOutTunable()
        .alongWith(mFeeder.setPercentOutTunable())
        .alongWith(mIntakeRollers.setTunable()));

        //unjam
        testPilot.leftBumper().whileTrue(
            mFloor.setPercentOutCommand(-0.5)
            .alongWith(mFeeder.setPercentOutCommand(-0.5)
            .alongWith(mIntakeRollers.setPercentOutCommand(-0.5))));

        //intake
       testPilot.rightBumper().whileTrue(mIntakeRollers.setTunable());

       // climb testing
       testPilot.x().whileTrue(
        mFastClimber.setPercentOutTunable()
       );
       
       testPilot.y().whileTrue(
        mFastClimber.setPercentOutTunableReverse()
       );

        testPilot.a().whileTrue(mFeeder.setPercentOutTunable());
        testPilot.b().whileTrue(mFloor.setPercentOutTunable());
        testPilot.back().whileTrue(mIntakeRollers.setTunable());

       //cowl
       testPilot.povUp().whileTrue(mCowl.setPositionTunable()); // Min 0 Max 1.8
       testPilot.povDown().onTrue(mCowl.home());

       //intakepivot
       testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
       testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());
    }

    private Command updateShooterVision() {
        return shooterLimelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;

            if (swerve.getState().Speeds.omegaRadiansPerSecond > 2) {
              return;
            }

            final Optional<Limelight.Measurement> measurement = shooterLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
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
            final Pose2d currentRobotPose = swerve.getState().Pose;

            if (swerve.getState().Speeds.omegaRadiansPerSecond > 2) {
              return;
            }

            final Optional<Limelight.Measurement> measurement = backLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    } 
}
