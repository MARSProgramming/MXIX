// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private static final double MAX_POSE_DIFF = 2.0; // meters
    private static final double MAX_TAG_DIST = 3; // reject poses further away than 3 meters.
    private static final double MIN_TAG_COUNT = 2; // reject pose estimates with less than 1 tag.
    private static final double FIELD_BORDER_MARGIN = 0.5; // meters
    private static final double MIN_TAG_AREA = 0.1; // percent
    private static final double MAX_LATENCY_SECONDS = 0.4; // 400ms
    

    private final CommandXboxController drivePilot = new CommandXboxController(0);
    private final CommandXboxController coPilot = new CommandXboxController(1);
    private final CommandXboxController testPilot = new CommandXboxController(2);
    private final Matrix<N3, N1> BACKCAM_TRUST = VecBuilder.fill(10.0, 10.0, 100);
    private final Matrix<N3, N1> SHOOTERCAM_TRUST = VecBuilder.fill(0.7, 0.7, 25);

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

       shooterLimelight.setDefaultCommand(integratedVisionUpdate());
    //  backLimelight.setDefaultCommand(updateBackVision());

      drivePilot.leftTrigger().whileTrue(mIntakeRollers.intakeCommand().beforeStarting(() -> leds.setColor(Color.kWhite, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightTrigger().whileTrue(
        new AimAndShootOnTheMove(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, 
        () -> -drivePilot.getLeftY(),  () -> -drivePilot.getLeftX())
        .beforeStarting(() -> leds.strobe(Color.kGreen, LEDSegment.ALL))
        .finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

      drivePilot.leftBumper().whileTrue(new Unjam(mFeeder, mFloor, mIntakeRollers).beforeStarting(() -> leds.setColor(Color.kRed, LEDSubsystem.LEDSegment.BOTH_BARS)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));
      drivePilot.rightBumper().whileTrue(new AimAndShuttle(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, () -> -drivePilot.getLeftY(), () -> -drivePilot.getLeftX()).beforeStarting(() -> leds.strobe(Color.kPurple, LEDSegment.ALL)).finallyDo(() -> leds.rainbow(LEDSegment.ALL)));

      drivePilot.povLeft().whileTrue(mIntakePivot.retractCommand());
      drivePilot.povRight().whileTrue(mIntakePivot.deployCommand());

      drivePilot.a().whileTrue(mIntakePivot.slamtake());
     drivePilot.b().whileTrue(new Shoot(swerve, mCowl, mFlywheel, mFeeder, mFloor, mIntakeRollers, leds, () -> -drivePilot.getLeftY(), () -> -drivePilot.getLeftX()));

     drivePilot.y().whileTrue(swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose)));
     drivePilot.x().onTrue(
        swerve.finalClimbLineupCommand().alongWith(shooterLimelight.idle()).withTimeout(3.5));



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

    private Command integratedVisionUpdate() {
    return Commands.run(() -> {
        final Pose2d currentRobotPose = swerve.getState().Pose;
        
        // Skip vision updates if rotating too fast
        if (swerve.getState().Speeds.omegaRadiansPerSecond > 2) {
            return;
        }
        
        // Try front camera first
        boolean frontCameraHasEstimate = processCameraMeasurement(
            shooterLimelight,
            "limelight-shooter",
            currentRobotPose,
            SHOOTERCAM_TRUST  // Use shootercam stdevs
        );
        
        // Only use back camera if front camera didn't have a valid measurement
        if (!frontCameraHasEstimate) {
            processCameraMeasurement(
                backLimelight,
                "limelight-back",
                currentRobotPose,
                BACKCAM_TRUST  // Use the higher std devs
            );
        }
        
    }, shooterLimelight, backLimelight)
    .ignoringDisable(true);
    }


    private boolean processCameraMeasurement(
    Limelight camera,
    String limelightName,
    Pose2d currentRobotPose,
    Matrix<N3, N1> overrideStdDevs
    ) {
    final Optional<Limelight.Measurement> measurement = camera.getMeasurement(currentRobotPose);
    
    if (measurement.isEmpty()) {
        return false;
    }

    if (measurement.get().poseEstimate == null
        || measurement.get().poseEstimate.pose.getTranslation() == null
        || !Double.isFinite(measurement.get().poseEstimate.pose.getTranslation().getX())
        || !Double.isFinite(measurement.get().poseEstimate.pose.getTranslation().getY())) {
            return false;
    }
    
    Pose2d measured = measurement.get().poseEstimate.pose;
    Rotation2d measuredRot = measured.getRotation();
    Rotation2d robotRot = currentRobotPose.getRotation();
    
    double rotationDifference = Math.abs(robotRot.minus(measuredRot).getRadians());
    double poseDiff = measured.getTranslation().getDistance(currentRobotPose.getTranslation());
    double latency = Timer.getFPGATimestamp() - measurement.get().poseEstimate.timestampSeconds;
    double targetArea = measurement.get().poseEstimate.avgTagArea;
    double tagDist = measurement.get().poseEstimate.avgTagDist;
    double tagCount = measurement.get().poseEstimate.tagCount;
    
    // Validation checks
    if (latency > MAX_LATENCY_SECONDS) return false;
    if (rotationDifference > MAX_ROTATION_DIFFERENCE) return false;
    if (targetArea < MIN_TAG_AREA) return false;
    if (poseDiff > MAX_POSE_DIFF)  return false;
    if (tagDist > MAX_TAG_DIST) return false;
    if (tagCount < MIN_TAG_COUNT) return false;
    
    // Field bounds check
    if (measured.getX() < -FIELD_BORDER_MARGIN
        || measured.getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
        || measured.getY() < -FIELD_BORDER_MARGIN
        || measured.getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN) {
        return false;
    }
    
    // Use override std devs if provided, otherwise use camera's std devs
    Matrix<N3, N1> stdDevs = overrideStdDevs != null ? overrideStdDevs : measurement.get().standardDeviations;
    
    swerve.addVisionMeasurement(
        measurement.get().poseEstimate.pose,
        measurement.get().poseEstimate.timestampSeconds,
        stdDevs
    );
    
    return true;  // Successfully added measurement
    }


    public LEDSubsystem getLedSubsystem() {
        return this.leds;
    }

}
