// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
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
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOnly;
import frc.robot.commands.Unjam;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;
import frc.robot.util.LimelightHelpers.RawFiducial;
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

    private final Vision vision;


    // Autonomous routines manager
    private final AutoRoutines autoRoutines = new AutoRoutines(swerve, mCowl, mFastClimber, mFeeder, mFloor, mFlywheel, mIntakePivot, mIntakeRollers, leds);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        configureTestBindings();
        autoRoutines.configure();

        vision =
        new Vision(
                swerve::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, () -> swerve.getState().Pose.getRotation()),
                new VisionIOLimelight(VisionConstants.camera1Name, () -> swerve.getState().Pose.getRotation()));

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


      drivePilot.leftTrigger().whileTrue(mIntakeRollers.intakeCommand().beforeStarting(() -> leds.setColor(Color.kWhite, LEDSubsystem.LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightTrigger().whileTrue(
        new AimAndShoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, 
        () -> -drivePilot.getLeftY(),  () -> -drivePilot.getLeftX())
        .beforeStarting(() -> leds.strobe(Color.kGreen, LEDSegment.ALL))
        .finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

      drivePilot.leftBumper().whileTrue(new Unjam(mFeeder, mFloor, mIntakeRollers).beforeStarting(() -> leds.setColor(Color.kRed, LEDSubsystem.LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightBumper().whileTrue(new AimAndShuttle(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, () -> -drivePilot.getLeftY(), () -> -drivePilot.getLeftX()).beforeStarting(() -> leds.strobe(Color.kPurple, LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

      drivePilot.povLeft().whileTrue(mIntakePivot.retractCommand());
      drivePilot.povRight().whileTrue(mIntakePivot.deployCommand());

      drivePilot.a().whileTrue(mIntakePivot.slamtake());
     drivePilot.b().whileTrue(new Shoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, () -> -drivePilot.getLeftY(), () -> -drivePilot.getLeftX()));

     drivePilot.y().whileTrue(swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose)));
     drivePilot.x().onTrue(swerve.finalClimbLineupCommand());



      drivePilot.povDown().whileTrue(mFastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE)
      .beforeStarting(() -> leds.strobe(Color.kRed, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.setColor(Color.kRed, LEDSegment.ALL)));
      drivePilot.povUp().whileTrue(mFastClimber.setPercentOut(-Settings.ClimbSettings.CLIMB_DUTYCYCLE)
            .beforeStarting(() -> leds.strobe(Color.kOrange, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.setColor(Color.kRed, LEDSegment.ALL)));


      // padcrafter: https://www.padcrafter.com/?templates=Pilot&leftStick=Translational+Control&rightStick=Rotational+Control&backButton=Reset+Gyro+Yaw&dpadUp=Run+Climber+Reverse&dpadDown=Run+Climber+Forward&dpadRight=Deploy+Intake&dpadLeft=Retract+Intake&aButton=Slamtake&rightTrigger=Aim+And+Shoot&rightBumper=Aim+And+Shuttle&bButton=Aim+and+Shoot+While+Moving&yButton=Align+to+Climb+Position&leftBumper=Unjam+&leftTrigger=Intake+
      // Manual feed on Copilot:  
      coPilot.leftTrigger().whileTrue(
        mFeeder.setPercentOutCommand(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE)
        .alongWith(mFloor.setPercentOutCommand(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE))
        .alongWith(mIntakeRollers.setPercentOutCommand(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE)));

      coPilot.leftBumper().whileTrue(mCowl.home());
        // Manual Hub base shot. Requires manual servoing of drivetrain.
      coPilot.rightTrigger().whileTrue(new ShootOnly(mCowl, mFlywheel, 
      Settings.ReferenceShotSettings.HUB_REFERENCE_FLYWHEEL_VELOCITY, 
      Settings.ReferenceShotSettings.HUB_REFERENCE_COWL_POSITION));

      coPilot.rightBumper().whileTrue(new ShootOnly(mCowl, mFlywheel, 
      Settings.ReferenceShotSettings.FRONT_OF_LADDER_REFERENCE_FLYWHEEL_VELOCITY, 
      Settings.ReferenceShotSettings.FRONT_OF_LADDER_REFERENCE_COWL_POSITION));

      // manual cowl homing
      coPilot.leftBumper().onTrue(
        mCowl.home()
      );

      coPilot.povDown().whileTrue(mFastClimber.setPercentOut(-Settings.ClimbSettings.CLIMB_DUTYCYCLE));

      // drivetrain unlock
      coPilot.x().onTrue(
        Commands.runOnce(
            () -> swerve.stop(), 
            swerve)
      );

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

       testPilot.back().whileTrue(
        mCowl.forwardTunable()
       );

       testPilot.start().whileTrue(
        mCowl.backwardTunable()
       );

        testPilot.a().whileTrue(mFeeder.setPercentOutTunable());
        testPilot.b().whileTrue(mFloor.setPercentOutTunable());

       //cowl
       testPilot.x().whileTrue(mCowl.setPositionTunable()); // Min 0 Max 1.8
       testPilot.y().onTrue(mCowl.home());

       //intakepivot
       testPilot.povRight().whileTrue(mIntakePivot.forwardTunable());
       testPilot.povLeft().whileTrue(mIntakePivot.backwardTunable());
    }


    public LEDSubsystem getLedSubsystem() {
        return this.leds;
    }

}
