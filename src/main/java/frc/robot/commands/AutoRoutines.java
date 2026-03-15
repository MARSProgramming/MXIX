// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.util.ChoreoTraj.B_BEELINE$0;
import static frc.robot.util.ChoreoTraj.B_BEELINE$1;
import static frc.robot.util.ChoreoTraj.B_BEELINE$2;
import static frc.robot.util.ChoreoTraj.B_BEELINE$3;
import static frc.robot.util.ChoreoTraj.C_BEELINE$0;
import static frc.robot.util.ChoreoTraj.C_BEELINE$1;
import static frc.robot.util.ChoreoTraj.C_BEELINE$2;
import static frc.robot.util.ChoreoTraj.C_BEELINE$3;
import static frc.robot.util.ChoreoTraj.C_DEPOT_ONETIME$0;
import static frc.robot.util.ChoreoTraj.C_DEPOT_ONETIME$1;
import static frc.robot.util.ChoreoTraj.C_DEPOT_ONETIME$2;
import static frc.robot.util.ChoreoTraj.X_CLIMB_NEARD$0;
import static frc.robot.util.ChoreoTraj.X_CLIMB_NEARD$1;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * Handles autonomous routine selection and configuration using Choreo.
 * This class manages the creation of auto routines and publishes the selector to the dashboard.
 */
public final class AutoRoutines {
    private final Swerve swerve;
    private final Cowl cowl;
    private final FastClimber fastClimber;
    private final Feeder feeder;
    private final Floor floor;
    private final Flywheel flywheel;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final LEDSubsystem ledsubsystem;
    
    private final Limelight shooterLimelight;
    private final Limelight backLimelight;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    /**
     * Creates a new AutoRoutines manager.
     *
     * @param swerve The swerve subsystem used for path following.
     * @param shooterLimelight The limelight used for shooter aiming.
     * @param backLimelight The back limelight.
     */
    public AutoRoutines(
        Swerve swerve,
        Cowl cowl,
        FastClimber fastClimber,
        Feeder feeder,
        Floor floor,
        Flywheel flywheel, 
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        LEDSubsystem ledsubsystem,
        Limelight shooterLimelight,
        Limelight backLimelight
    ) {
        this.swerve = swerve;
        this.cowl = cowl;
        this.fastClimber = fastClimber;
        this.feeder = feeder;
        this.floor = floor;
        this.flywheel = flywheel;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.shooterLimelight = shooterLimelight;
        this.backLimelight = backLimelight;
        this.ledsubsystem = ledsubsystem;
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("Depot Bump Score Beeline", this::CBeeline);
        autoChooser.addRoutine("Outpost Bump Score Beeline", this::BBeeline);
        autoChooser.addRoutine("Depot Bump Score Depot", this::CDepot);
        autoChooser.addRoutine("Middle Score Preload Climb", this::XClimbNearDepot);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Schedule the selected autonomous command when the robot enters autonomous mode
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler()
        .beforeStarting(() -> ledsubsystem.setColor(Color.kRed, LEDSegment.ALL))
        .finallyDo(() -> ledsubsystem.rainbow(LEDSegment.ALL)));
    }



    private AutoRoutine CBeeline() {
        final AutoRoutine routine = autoFactory.newRoutine("C_BEELINE_ROUTINE");
        final AutoTrajectory goOverBumpTraj = C_BEELINE$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = C_BEELINE$1.asAutoTraj(routine);
        final AutoTrajectory sweepBallsTraj = C_BEELINE$2.asAutoTraj(routine);
        final AutoTrajectory returnToShootTraj = C_BEELINE$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
                .beforeStarting(() -> ledsubsystem.rainbow(LEDSegment.ALL))
            )
        );

        goOverBumpTraj.done().onTrue(prepSweep.cmd().alongWith(intakePivot.timedDeployCommand()));
        prepSweep.done().onTrue(sweepBallsTraj.cmd().alongWith(intakeRollers.intakeCommand()));

        sweepBallsTraj.done().onTrue(returnToShootTraj.cmd());

        returnToShootTraj.done().onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem)
            .alongWith(intakePivot.slamtake())
        );

        return routine;
    }

    
    private AutoRoutine BBeeline() {
        final AutoRoutine routine = autoFactory.newRoutine("B_BEELINE_ROUTINE");
        final AutoTrajectory goOverBumpTraj = B_BEELINE$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = B_BEELINE$1.asAutoTraj(routine);
        final AutoTrajectory sweepBallsTraj = B_BEELINE$2.asAutoTraj(routine);
        final AutoTrajectory returnToShootTraj = B_BEELINE$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
                .beforeStarting(() -> ledsubsystem.rainbow(LEDSegment.ALL))
            )
        );

        goOverBumpTraj.done().onTrue(prepSweep.cmd().alongWith(intakePivot.timedDeployCommand()));
        prepSweep.done().onTrue(sweepBallsTraj.cmd().alongWith(intakeRollers.intakeCommand()));

        sweepBallsTraj.done().onTrue(returnToShootTraj.cmd());

        returnToShootTraj.done().onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem)
            .alongWith(intakePivot.slamtake())
        );

        return routine;
    }

    private AutoRoutine CDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("C_DEPOT_ROUTINE");
        final AutoTrajectory getPoseTraj = C_DEPOT_ONETIME$0.asAutoTraj(routine);
        final AutoTrajectory gettingBalls = C_DEPOT_ONETIME$1.asAutoTraj(routine);
        final AutoTrajectory rotateToScore = C_DEPOT_ONETIME$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            getPoseTraj.resetOdometry(),
            getPoseTraj.cmd().alongWith(intakePivot.timedDeployCommand())
            )
        );

        getPoseTraj.done().onTrue(gettingBalls.cmd().alongWith(intakeRollers.intakeCommand()));

        gettingBalls.done().onTrue(rotateToScore.cmd());
        rotateToScore.done().onTrue(new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem)
        .alongWith(intakePivot.slamtake()));

        return routine;
    }



    

    private AutoRoutine XClimbNearDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("X_CLIMB_DEPOT_ROUTINE");
        final AutoTrajectory goToShotPos = X_CLIMB_NEARD$0.asAutoTraj(routine);
        final AutoTrajectory prelineupClimb = X_CLIMB_NEARD$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            goToShotPos.resetOdometry(),
            goToShotPos.cmd().alongWith(intakePivot.timedDeployCommand())             
            ) 
        );

        goToShotPos.done().onTrue(
            Commands.parallel(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).withTimeout(4),
                intakePivot.slamtake().withTimeout(4)   
            )
        .andThen(prelineupClimb.cmd()));


        prelineupClimb.doneDelayed(1).onTrue(
            swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose))
        .withTimeout(1.5)
        .andThen(
            swerve.finalClimbLineupCommand().withTimeout(3.7)
            .andThen(
                fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
            )
            ));


        return routine;

    }
}