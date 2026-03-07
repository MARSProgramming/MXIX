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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
    private DrivetrainTelemetry dtTelem = new DrivetrainTelemetry(swerve);

    FastClimber fastClimb = new FastClimber();
    private final AutoRoutines autoRoutines = new AutoRoutines(swerve,
            shooterLimelight, backLimelight);

    Command prepShotCommand = new PrepareSupershot(
            shotSetup,
            swerve,
            mFlywheel,
            mCowl,
            () -> -drivePilot.getLeftY(),
            () -> -drivePilot.getLeftX());

    // Superstructure mSuperstructure = new Superstructure(mCowl, mFlywheel,
    // mFeeder, mFloor);

    // DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        // Handles autonomous command selection and configuration. Deprecates
        // getAutonomousCommand() generated method
        autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
                swerve,
                () -> -testPilot.getLeftY(),
                () -> -testPilot.getLeftX());

        testPilot.leftTrigger().whileTrue(mIntakeRollers.setTunable());

        // testPilot.rightTrigger().whileTrue(
        //         mFlywheel.setVelocity(() -> shotSetup.getStaticShotInfo(swerve.getDistanceToHub()).shot.shooterRPM)
        //                 .alongWith(mCowl.setPositionCommand(
        //                         () -> shotSetup.getStaticShotInfo(swerve.getDistanceToHub()).cowlPosition)));
        // testPilot.rightTrigger().whileTrue(aimAndDriveCommand);
         testPilot.rightTrigger().whileTrue(mFlywheel.setVelocityTunable());

        testPilot.rightBumper().whileTrue(mFloor.setPercentOutTunable().alongWith(mFeeder.setPercentOutTunable())
                .alongWith(mIntakeRollers.setTunable()));
        testPilot.b().onTrue(mCowl.home());

        testPilot.leftBumper()
                .whileTrue(mFloor.set(-0.5).alongWith(mFeeder.setPercentOut(-0.5).alongWith(mIntakeRollers.set(-0.5))));

        testPilot.a().whileTrue(fastClimb.setPercentOutTunable());
        testPilot.y().whileTrue(fastClimb.setPercentOutTunableReverse());

        testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
        testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());

        testPilot.povUp().whileTrue(mCowl.forwardTunable());
        testPilot.povDown().whileTrue(mCowl.backwardTunable());

        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
                swerve,
                () -> -testPilot.getLeftY(),
                () -> -testPilot.getLeftX(),
                () -> -testPilot.getRightX());

        swerve.setDefaultCommand(manualDriveCommand);
        shooterLimelight.setDefaultCommand(updateShooterVision());
        backLimelight.setDefaultCommand(updateBackVision());

        testPilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));

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
                        m.standardDeviations);
            });
        })
                .ignoringDisable(true);
    }

    /**
     * Creates a command to continuously update the robot's pose using the back
     * Limelight.
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
                        m.standardDeviations);
            });
        })
                .ignoringDisable(true);
    }

}
