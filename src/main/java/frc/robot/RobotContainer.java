// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SystemConstants;
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
import frc.robot.util.GeometryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Maximum speed of the robot in meters per second, derived from TunerConstants.

    private final Limelight shooterLimelight = new Limelight(SystemConstants.Limelights.kShooterLimelightName);
    private final Limelight backLimelight = new Limelight(SystemConstants.Limelights.kBackLimelightName);


    // The driver's controller.
    private final CommandXboxController drivePilot = new CommandXboxController(0);
    private final CommandXboxController testPilot = new CommandXboxController(1);

    // Subsystems
    Cowl mCowl = new Cowl();
    Flywheel mFlywheel = new Flywheel();
    Feeder mFeeder = new Feeder();
    Floor mFloor = new Floor();
    IntakePivot mIntakePivot = new IntakePivot();
    Swerve mSwerve = new Swerve();
    IntakeRollers mIntakeRollers = new IntakeRollers();
    FastClimber fastClimb = new FastClimber();

    Superstructure mSuperstructure = new Superstructure(mCowl, mSwerve, mFeeder, mFloor, fastClimb, mFlywheel, mIntakePivot, mIntakeRollers, shooterLimelight, backLimelight);
    
    private final AutoRoutines autoRoutines = new AutoRoutines(mSuperstructure);

    public RobotContainer() {
        configureBindings();
        configureTestBindings();
        autoRoutines.configure();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    public void configureBindings() {
    final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
          mSuperstructure.getSwerveSubsystem(),
          () -> -drivePilot.getLeftY(),
          () -> -drivePilot.getLeftX(),
          () -> -drivePilot.getRightX());

    final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
            mSuperstructure.getSwerveSubsystem(), 
            () -> -drivePilot.getLeftY(), 
            () -> -drivePilot.getLeftX());


      drivePilot.leftTrigger().whileTrue(mSuperstructure.intakeCommand());
      drivePilot.rightTrigger().whileTrue(
        mSuperstructure.aimAndStaticShot(
            () -> -drivePilot.getLeftY(), 
            () -> -drivePilot.getLeftX()
        )
      );

      drivePilot.leftBumper().whileTrue(mSuperstructure.unjamCommand());
      drivePilot.rightBumper().whileTrue(mSuperstructure.feedCommand());
      //drivePilot.rightBumper().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(FieldConstants.Orientations.getClosestDiamond(mSuperstructure.getSwerveSubsystem().getState().Pose)))); 

      drivePilot.povRight().onTrue(mSuperstructure.getIntakePivotSubsystem().timedDeployCommand());
      drivePilot.povLeft().onTrue(mSuperstructure.getIntakePivotSubsystem().timedRetractCommand());


      shooterLimelight.setDefaultCommand(updateShooterVision());
      mSuperstructure.getSwerveSubsystem().setDefaultCommand(manualDriveCommand);
      mSuperstructure.getBackLimelight().setDefaultCommand(updateBackVision());
      drivePilot.leftBumper().whileTrue(aimAndDriveCommand);


      drivePilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    public void configureTestBindings() {
        // shooting
        testPilot.rightTrigger().whileTrue(mSuperstructure.getFlywheelSubsystem().setVelocityTunable());
        testPilot.leftTrigger().whileTrue(
        mSuperstructure.getFloorSubsystem().setPercentOutTunable()
        .alongWith(mSuperstructure.getFeederSubsystem().setPercentOutTunable())
        .alongWith(mSuperstructure.getIntakeRollersSubsystem().setTunable()));

        //unjam
        testPilot.leftBumper().whileTrue(
            mSuperstructure.getFloorSubsystem().set(-0.5)
            .alongWith(mSuperstructure.getFeederSubsystem().setPercentOut(-0.5)
            .alongWith(mSuperstructure.getIntakeRollersSubsystem().set(-0.5))));

        //intake
       testPilot.rightBumper().whileTrue(mIntakeRollers.setTunable());

       // climb testing
       testPilot.x().whileTrue(
        mSuperstructure.getFastClimberSubsystem().setPercentOutTunable()
       );
       
       testPilot.y().whileTrue(
        mSuperstructure.getFastClimberSubsystem().setPercentOutTunableReverse()
       );

        testPilot.a().whileTrue(mSuperstructure.getFeederSubsystem().setPercentOutTunable());
        testPilot.b().whileTrue(mSuperstructure.getFloorSubsystem().setPercentOutTunable());

        testPilot.back().whileTrue(mSuperstructure.getIntakeRollersSubsystem().setTunable());
       //cowl
       testPilot.povUp().whileTrue(mCowl.setPositionTunable()); // Min 0 Max 1.8
       testPilot.povDown().onTrue(mCowl.home());

       //intakepivot
       testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
       testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());
    }

    private Command updateShooterVision() {
        return mSuperstructure.getShooterLimelight().run(() -> {
            final Pose2d currentRobotPose = mSuperstructure.getSwerveSubsystem().getState().Pose;

            final Optional<Limelight.Measurement> measurement = shooterLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {

                // Discard measurement if we're rotating too fast
                if (mSuperstructure.getSwerveSubsystem().getState().Speeds.omegaRadiansPerSecond > 2.0)  { 
                    return; 
                }
                // Discard measurements that are outside the field boundaries
                if (!GeometryUtil.isInField(m.poseEstimate.pose)) { 
                    return; 
                }
                // Discard invalid rotation measurements
                if (Math.abs(m.poseEstimate.pose.getRotation().minus(mSuperstructure.getSwerveSubsystem().getState().Pose.getRotation()).getDegrees()) > 45) { 
                    return; 
                }
                mSuperstructure.getSwerveSubsystem().addVisionMeasurement(
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
        return mSuperstructure.getBackLimelight().run(() -> {
            final Pose2d currentRobotPose = mSuperstructure.getSwerveSubsystem().getState().Pose;

            if (mSuperstructure.getSwerveSubsystem().getState().Speeds.omegaRadiansPerSecond > 2) {
              return;
            }

            final Optional<Limelight.Measurement> measurement = backLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                mSuperstructure.getSwerveSubsystem().addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    } 
}
