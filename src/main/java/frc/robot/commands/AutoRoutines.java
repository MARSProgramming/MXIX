// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.util.ChoreoTraj.B_TEST$0;
import static frc.robot.util.ChoreoTraj.B_TEST$1;
import static frc.robot.util.ChoreoTraj.B_TEST$2;
import static frc.robot.util.ChoreoTraj.B_TEST$3;
import static frc.robot.util.ChoreoTraj.C_CLIMB_NEARD$0;
import static frc.robot.util.ChoreoTraj.C_CLIMB_NEARD$1;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$0;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$1;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$2;
import static frc.robot.util.ChoreoTraj.C_TEST;
import static frc.robot.util.ChoreoTraj.C_TEST$0;
import static frc.robot.util.ChoreoTraj.C_TEST$1;
import static frc.robot.util.ChoreoTraj.C_TEST$2;
import static frc.robot.util.ChoreoTraj.C_TEST$3;
import static frc.robot.util.ChoreoTraj.X_CLIMB_NEARD$0;
import static frc.robot.util.ChoreoTraj.X_CLIMB_NEARD$1;
import static frc.robot.util.ChoreoTraj.X_DEPOT_AND_CLIMB$0;
import static frc.robot.util.ChoreoTraj.X_DEPOT_AND_CLIMB$1;
import static frc.robot.util.ChoreoTraj.X_DEPOT_AND_CLIMB$2;

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
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ShotSetup;
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
    private final ShotSetup shotSetup;
    

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
        ShotSetup shotSetup
    ) {
        this.swerve = swerve;
        this.cowl = cowl;
        this.fastClimber = fastClimber;
        this.feeder = feeder;
        this.floor = floor;
        this.flywheel = flywheel;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.ledsubsystem = ledsubsystem;
        this.shotSetup = shotSetup;
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("DepotBump Score Preload Climb", this::CClimbNearDepot);
        autoChooser.addRoutine("DepotBump Score Depot Climb", this::CScoreDepotClimb);
        autoChooser.addRoutine("Middle Score Preload Climb", this::XClimbNearDepot);
        autoChooser.addRoutine("Middle Score Depot Climb", this::XScoreDepotClimb);
        autoChooser.addRoutine("Middle Reset Odom", this::XClimbResetOdom);
        autoChooser.addRoutine("Depot Bump Sweep Return", this::CContinuousSweep);
        autoChooser.addRoutine("Outpost Bump Sweep Return", this::BContinuousSweep);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Schedule the selected autonomous command when the robot enters autonomous mode
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler()
        .beforeStarting((() -> ledsubsystem.setColor(Color.kRed, LEDSegment.ALL)))
        .finallyDo(() -> ledsubsystem.rainbow(LEDSegment.ALL)));
    }


    private AutoRoutine CContinuousSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("C_SWEEP_ROUTINE");
        final AutoTrajectory goOverBump = C_TEST$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = C_TEST$1.asAutoTraj(routine);
        final AutoTrajectory sweep = C_TEST$2.asAutoTraj(routine);
        final AutoTrajectory bumpTraversal = C_TEST$3.asAutoTraj(routine);
        

        routine.active().onTrue(
            Commands.sequence(
                goOverBump.resetOdometry(),
                goOverBump.cmd().alongWith(intakePivot.timedDeployCommand())
            )
        );

        goOverBump.done().onTrue(prepSweep.cmd());

        prepSweep.done().onTrue(
            sweep.cmd().alongWith(intakeRollers.intakeCommand())
            .until(() -> sweep.done().getAsBoolean())
        );

        sweep.done().onTrue(
            bumpTraversal.cmd()
        );

        bumpTraversal.done().onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup)
            .alongWith(intakePivot.slamtake())
        );

        return routine;
    }

    
    private AutoRoutine BContinuousSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("C_SWEEP_ROUTINE");
        final AutoTrajectory goOverBump = B_TEST$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = B_TEST$1.asAutoTraj(routine);
        final AutoTrajectory sweep = B_TEST$2.asAutoTraj(routine);
        final AutoTrajectory bumpTraversal = B_TEST$3.asAutoTraj(routine);
        

        routine.active().onTrue(
            Commands.sequence(
                goOverBump.resetOdometry(),
                goOverBump.cmd().alongWith(intakePivot.timedDeployCommand())
            )
        );

        goOverBump.done().onTrue(prepSweep.cmd());

        prepSweep.done().onTrue(
            sweep.cmd().alongWith(intakeRollers.intakeCommand())
            .until(() -> sweep.done().getAsBoolean())
        );

        sweep.done().onTrue(
            bumpTraversal.cmd()
        );

        bumpTraversal.done().onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup)
            .alongWith(intakePivot.slamtake())
        );
        
        return routine;
    }
    
    
    

    private AutoRoutine XClimbResetOdom() {
        final AutoRoutine routine = autoFactory.newRoutine("X_CLIMB_DEPOT_ROUTINE");
        final AutoTrajectory goToShotPos = X_CLIMB_NEARD$0.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            goToShotPos.resetOdometry()
            ) 
        );
        
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

    
        goToShotPos.doneDelayed(0.5).onTrue(
            Commands.parallel(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup).withTimeout(4),
                intakePivot.slamtake().withTimeout(3.8)   
            ));

        goToShotPos.doneDelayed(3.9).onTrue(prelineupClimb.cmd().alongWith(intakePivot.confirmDeploy()));

        prelineupClimb.done().onTrue(
            intakePivot.confirmDeploy()
        );

        prelineupClimb.doneDelayed(0.5).onTrue(
            swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose))
        .withTimeout(1)
        .andThen(
            swerve.finalClimbLineupCommand().withTimeout(3.7)
            .andThen(
                fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
            )
            ));


        return routine;

    }

        private AutoRoutine CClimbNearDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("C_CLIMB_DEPOT_ROUTINE");
        final AutoTrajectory goToShotPos = C_CLIMB_NEARD$0.asAutoTraj(routine);
        final AutoTrajectory prelineupClimb = C_CLIMB_NEARD$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            goToShotPos.resetOdometry(),
            goToShotPos.cmd().alongWith(intakePivot.timedDeployCommand())             
            ) 
        );

    
        goToShotPos.doneDelayed(0.5).onTrue(
            Commands.parallel(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup).withTimeout(4),
                intakePivot.slamtake().withTimeout(3.8)   
            ));

        goToShotPos.doneDelayed(3.9).onTrue(prelineupClimb.cmd().alongWith(intakePivot.confirmDeploy()));

        prelineupClimb.done().onTrue(
            intakePivot.confirmDeploy()
        );

        prelineupClimb.doneDelayed(0.5).onTrue(
            swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose))
        .withTimeout(1)
        .andThen(
            swerve.finalClimbLineupCommand().withTimeout(3.7)
            .andThen(
                fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
            )
            ));


        return routine;

    }
    

    private AutoRoutine XScoreDepotClimb() {
        final AutoRoutine routine = autoFactory.newRoutine("X_SCORE_DEPOT_CLIMB_ROUTINE");
        final AutoTrajectory getTagPos = X_DEPOT_AND_CLIMB$0.asAutoTraj(routine);
        final AutoTrajectory getBallsTraj = X_DEPOT_AND_CLIMB$1.asAutoTraj(routine);
        final AutoTrajectory prelineupClimb = X_DEPOT_AND_CLIMB$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            getTagPos.resetOdometry(),
            getTagPos.cmd().alongWith(intakePivot.timedDeployCommand())             
            ) 
        );

        getTagPos.doneDelayed(0.5).onTrue(
            getBallsTraj.cmd().alongWith(intakeRollers.intakeCommand())
        );

        getBallsTraj.done().onTrue(
            Commands.parallel(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup).withTimeout(4),
                intakePivot.slamtake().withTimeout(3.8)   
            ));

        getBallsTraj.doneDelayed(4.0).onTrue(prelineupClimb.cmd().alongWith(intakePivot.confirmDeploy()));

        prelineupClimb.done()
        .onTrue(
            intakePivot.confirmDeploy()
        );

        prelineupClimb.doneDelayed(0.2).onTrue(
            swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose))
        .withTimeout(1)
        .andThen(
            swerve.finalClimbLineupCommand().withTimeout(3.7)
           // .andThen(
          //      fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
          //  )
            ));

        return routine;
    }


    
    private AutoRoutine CScoreDepotClimb() {
        final AutoRoutine routine = autoFactory.newRoutine("C_SCORE_DEPOT_CLIMB_ROUTINE");
        final AutoTrajectory getTagPos = C_DEPOT_AND_CLIMB$0.asAutoTraj(routine);
        final AutoTrajectory getBallsTraj = C_DEPOT_AND_CLIMB$1.asAutoTraj(routine);
        final AutoTrajectory prelineupClimb = C_DEPOT_AND_CLIMB$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
            getTagPos.resetOdometry(),
            getTagPos.cmd().alongWith(intakePivot.timedDeployCommand())             
            ) 
        );

        getTagPos.doneDelayed(0.5).onTrue(
            getBallsTraj.cmd().alongWith(intakeRollers.intakeCommand())
        );

        getBallsTraj.done().onTrue(
            Commands.parallel(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup).withTimeout(4),
                intakePivot.slamtake().withTimeout(3.8)   
            ));

        getBallsTraj.doneDelayed(4.0).onTrue(prelineupClimb.cmd().alongWith(intakePivot.confirmDeploy()));

        prelineupClimb.doneDelayed(0.2).onTrue(
            swerve.alignToPoint(() -> FieldConstants.getClosestClimbingPosition(swerve.getState().Pose))
        .withTimeout(1)
        .andThen(
            swerve.finalClimbLineupCommand().withTimeout(3.7)
            .andThen(
                fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
            )
            ));

        return routine;
    }
}