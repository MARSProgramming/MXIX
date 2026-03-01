// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.reflect.Field;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;
import frc.robot.util.GeometryUtil;
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

    ShotSetup setup = new ShotSetup();

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
    FastClimber fastClimb = new FastClimber();

    Superstructure mSuperstructure = new Superstructure(mCowl, swerve, mFeeder, mFloor, fastClimb, mFlywheel, mIntakePivot, mIntakeRollers);

    private DrivetrainTelemetry dtTelem = new DrivetrainTelemetry(swerve);

    public RobotContainer() {
        configureBindings();
        // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
     //  autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
    final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
          swerve,
          () -> -testPilot.getLeftY(),
          () -> -testPilot.getLeftX(),
          () -> -testPilot.getRightX());

    final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
            swerve, 
            () -> -testPilot.getLeftY(), 
            () -> -testPilot.getLeftX());


      testPilot.leftTrigger().whileTrue(mSuperstructure.intakeCommand());
      testPilot.rightTrigger().whileTrue(
        mSuperstructure.aimAndStaticShot(
            () -> -testPilot.getLeftY(), 
            () -> -testPilot.getLeftX()
        )
      );

      testPilot.leftBumper().whileTrue(mSuperstructure.unjamCommand());
      testPilot.rightBumper().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(FieldConstants.Orientations.getClosestDiamond(swerve.getState().Pose)))); 

      testPilot.povRight().whileTrue(mIntakePivot.deployCommand());
      testPilot.povLeft().whileTrue(mIntakePivot.retractCommand());



      swerve.setDefaultCommand(manualDriveCommand);
      shooterLimelight.setDefaultCommand(updateShooterVision());
    //  backLimelight.setDefaultCommand(updateBackVision());
     testPilot.leftBumper().whileTrue(aimAndDriveCommand);


      testPilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));



    }

    private Command updateShooterVision() {
        return shooterLimelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;

            final Optional<Limelight.Measurement> measurement = shooterLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {

                // Discard measurement if we're rotating too fast
                if (swerve.getState().Speeds.omegaRadiansPerSecond > 2.0)  { 
                    return; 
                }
                // Discard measurements that are outside the field boundaries
                if (!GeometryUtil.isInField(m.poseEstimate.pose)) { 
                    return; 
                }
                // Discard invalid rotation measurements
                if (Math.abs(m.poseEstimate.pose.getRotation().minus(swerve.getState().Pose.getRotation()).getDegrees()) > 45) { 
                    return; 
                }
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

     /*
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
    } */
}
