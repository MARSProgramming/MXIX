// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimAndShootOnTheMove;
import frc.robot.commands.AimAndShuttle;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.ShootOnly;
import frc.robot.commands.Unjam;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Settings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;
import frc.robot.util.LimelightHelpers;
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


    private static final double MAX_ROTATION_DIFFERENCE = Math.toRadians(30); // radians

    // Vision filtering constants
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters
    private static final double MIN_TAG_AREA = 0.1; // percent
    private static final double MAX_LATENCY_SECONDS = 0.4; // 400ms
    

    private final CommandXboxController drivePilot = new CommandXboxController(0);
    private final CommandXboxController coPilot = new CommandXboxController(1);
    private final CommandXboxController testPilot = new CommandXboxController(2);

    // Subsystems
    Cowl mCowl = new Cowl();
    Flywheel mFlywheel = new Flywheel();
    Feeder mFeeder = new Feeder();
    Floor mFloor = new Floor();
    IntakePivot mIntakePivot = new IntakePivot();
    ShotSetup shotSetup = new ShotSetup();
    Swerve swerve = new Swerve();
    IntakeRollers mIntakeRollers = new IntakeRollers();
    LEDSubsystem leds = new LEDSubsystem();
    FastClimber mFastClimber = new FastClimber();




    // Autonomous routines manager
    private final AutoRoutines autoRoutines = new AutoRoutines(swerve, mCowl, mFastClimber, mFeeder, mFloor, mFlywheel, mIntakePivot, mIntakeRollers, leds, shooterLimelight, backLimelight);
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
       //backLimelight.setDefaultCommand(updateBackVision());

      drivePilot.leftTrigger().whileTrue(mIntakeRollers.intakeCommand().beforeStarting(() -> leds.setColor(Color.kWhite, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightTrigger().whileTrue(
        new AimAndShoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, 
        () -> -drivePilot.getLeftY(),  () -> -drivePilot.getLeftX())
        .beforeStarting(() -> leds.strobe(Color.kGreen, LEDSegment.ALL))
        .finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

       drivePilot.b().whileTrue(
        new AimAndShoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, 
        () -> -drivePilot.getLeftY(),  () -> -drivePilot.getLeftX())
        .beforeStarting(() -> leds.strobe(Color.kGreen, LEDSegment.ALL))
        .finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.leftBumper().whileTrue(new Unjam(mFeeder, mFloor, mIntakeRollers).beforeStarting(() -> leds.setColor(Color.kRed, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightBumper().whileTrue(new AimAndShuttle(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, () -> -drivePilot.getLeftY(), () -> -drivePilot.getLeftX()).beforeStarting(() -> leds.strobe(Color.kPurple, LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

      drivePilot.povLeft().whileTrue(mIntakePivot.retractCommand());
      drivePilot.povRight().whileTrue(mIntakePivot.deployCommand());

      drivePilot.x().whileTrue(mIntakePivot.slamtake());

      drivePilot.povDown().whileTrue(mFastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE)
      .beforeStarting(() -> leds.strobe(Color.kRed, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.setColor(Color.kRed, LEDSegment.ALL)));
      drivePilot.povUp().whileTrue(mFastClimber.setPercentOut(-Settings.ClimbSettings.CLIMB_DUTYCYCLE)
            .beforeStarting(() -> leds.strobe(Color.kOrange, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.setColor(Color.kRed, LEDSegment.ALL)));


      // Manual feed on Copilot:  
      coPilot.leftBumper().whileTrue(
        mFeeder.setPercentOutCommand(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE)
        .alongWith(mFloor.setPercentOutCommand(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE))
        .alongWith(mIntakeRollers.setPercentOutCommand(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE)));

        // Manual Hub base shot. Requires manual servoing of drivetrain.
      coPilot.rightTrigger().whileTrue(new ShootOnly(mCowl, mFlywheel, 
      Settings.ReferenceShotSettings.HUB_REFERENCE_FLYWHEEL_VELOCITY, 
      Settings.ReferenceShotSettings.HUB_REFERENCE_COWL_POSITION));

      // manual cowl homing
      coPilot.leftBumper().onTrue(
        mCowl.home()
      );

      coPilot.x().whileTrue(swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose)));
      coPilot.y().onTrue(swerve.finalClimbLineupCommand().beforeStarting(() -> leds.fadeColor(Color.kPurple, LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

    }

    private void configureTestBindings() {
        // shooting
        testPilot.rightTrigger().whileTrue(mFlywheel.setVelocityTunable());
        testPilot.rightBumper().whileTrue(
        mFloor.setPercentOutTunable()
        .alongWith(mFeeder.setPercentOutTunable())
        .alongWith(mIntakeRollers.setTunable()));

        //unjam
        testPilot.leftBumper().whileTrue(
            mFloor.setPercentOutCommand(-0.5)
            .alongWith(mFeeder.setPercentOutCommand(-0.5)
            .alongWith(mIntakeRollers.setPercentOutCommand(-0.5))));

        //intake
       testPilot.leftTrigger().whileTrue(mIntakeRollers.setTunable());

       // climb testing
       testPilot.povDown().whileTrue(
        mFastClimber.setPercentOutTunable()
       );
       
       testPilot.povUp().whileTrue(
        mFastClimber.setPercentOutTunableReverse()
       );

        testPilot.a().whileTrue(mFeeder.setPercentOutTunable());
        testPilot.b().whileTrue(mFloor.setPercentOutTunable());
        testPilot.back().whileTrue(mIntakeRollers.setTunable());

       //cowl
       testPilot.x().whileTrue(mCowl.setPositionTunable()); // Min 0 Max 1.8
       testPilot.y().onTrue(mCowl.home());

       //intakepivot
       testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
       testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());
    }

    private Command updateShooterVision() {
        return shooterLimelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = shooterLimelight.getMeasurement(currentRobotPose);
            if (swerve.getState().Speeds.omegaRadiansPerSecond > 2) {
              return;
            }
            measurement.ifPresent(m -> {

                Pose2d measured = measurement.get().poseEstimate.pose;
                Rotation2d measuredRot = measurement.get().poseEstimate.pose.getRotation();
                Rotation2d robotRot = swerve.getState().Pose.getRotation();

                double rotationDifference = Math.abs(robotRot.minus(measuredRot).getRadians());
                double latency = Timer.getFPGATimestamp() - measurement.get().poseEstimate.timestampSeconds;
                double targetArea = LimelightHelpers.getTA("limelight-shooter");

                if (latency > MAX_LATENCY_SECONDS) {
                    return;
                }

                if (rotationDifference > MAX_ROTATION_DIFFERENCE) {
                 return;
                }
        
                if (targetArea < MIN_TAG_AREA) {
                return;
                }

                 // 1. Reject if robot pose is off the field
                if (
                measured.getX() < -FIELD_BORDER_MARGIN
                || measured.getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
                || measured.getY() < -FIELD_BORDER_MARGIN
                || measured.getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN) {
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

    private Command updateBackVision() {
        return backLimelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = backLimelight.getMeasurement(currentRobotPose);
            if (swerve.getState().Speeds.omegaRadiansPerSecond > 2) {
              return;
            }
            measurement.ifPresent(m -> {

                Pose2d measured = measurement.get().poseEstimate.pose;
                Rotation2d measuredRot = measurement.get().poseEstimate.pose.getRotation();
                Rotation2d robotRot = swerve.getState().Pose.getRotation();

                double rotationDifference = Math.abs(robotRot.minus(measuredRot).getRadians());
                double latency = Timer.getFPGATimestamp() - measurement.get().poseEstimate.timestampSeconds;
                double targetArea = LimelightHelpers.getTA("limelight-shooter");

                if (latency > MAX_LATENCY_SECONDS) {
                    return;
                }

                if (rotationDifference > MAX_ROTATION_DIFFERENCE) {
                 return;
                }
        
                if (targetArea < MIN_TAG_AREA) {
                return;
                }

                 // 1. Reject if robot pose is off the field
                if (
                measured.getX() < -FIELD_BORDER_MARGIN
                || measured.getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
                || measured.getY() < -FIELD_BORDER_MARGIN
                || measured.getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN) {
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

    public LEDSubsystem getLedSubsystem() {
        return this.leds;
    }

}
