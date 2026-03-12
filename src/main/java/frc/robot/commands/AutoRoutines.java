// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.ChoreoTraj.C_BEELINE_FAMILY$0;
import static frc.robot.util.ChoreoTraj.C_BEELINE_FAMILY$1;
import static frc.robot.util.ChoreoTraj.C_BEELINE_FAMILY$2;
import static frc.robot.util.ChoreoTraj.C_BEELINE_FAMILY$3;
import static frc.robot.util.ChoreoTraj.C_HOME_FAMILY$0;
import static frc.robot.util.ChoreoTraj.C_HOME_FAMILY$1;
import static frc.robot.util.ChoreoTraj.C_HOME_FAMILY$2;
import static frc.robot.util.ChoreoTraj.D_HOME_FAMILY$0;
import static frc.robot.util.ChoreoTraj.D_HOME_FAMILY$1;
import static frc.robot.util.ChoreoTraj.D_HOME_FAMILY$2;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Settings;
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
        Limelight shooterLimelight
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
        this.ledsubsystem = ledsubsystem;
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("C_BEELINE", this::C_BEELINE);
        autoChooser.addRoutine("C_BEELINE_GREED", this::C_BEELINE_GREED);
        autoChooser.addRoutine("C_PRELOAD", this::C_PRELOAD);
        autoChooser.addRoutine("C_DEPOT", this::C_DEPOT);
        autoChooser.addRoutine("D_PRELOAD", this::D_PRELOAD);
        autoChooser.addRoutine("D_DEPOT", this::D_DEPOT);



        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Schedule the selected autonomous command when the robot enters autonomous mode
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }



    private AutoRoutine C_BEELINE() {
        final AutoRoutine routine = autoFactory.newRoutine("C_BEELINE_ROUTINE");
        final AutoTrajectory goOverBumpTraj = C_BEELINE_FAMILY$0.asAutoTraj(routine);
        final AutoTrajectory getBallsInCenterTraj = C_BEELINE_FAMILY$1.asAutoTraj(routine);
        final AutoTrajectory returnToShoot = C_BEELINE_FAMILY$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
            )
        );

        goOverBumpTraj.done().onTrue(intakePivot.deployCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT));
        goOverBumpTraj.done().onTrue(getBallsInCenterTraj.cmd().alongWith(intakeRollers.intakeCommand()
        .alongWith(floor.setPercentOutCommand(Settings.IntakeSystemSettings.INTAKING_FLOOR_DUTYCYCLE)
        .alongWith(feeder.setPercentOutCommand(-Settings.IntakeSystemSettings.INTAKING_FEEDER_DUTYCYCLE)))));

        getBallsInCenterTraj.done().onTrue(returnToShoot.cmd());

        returnToShoot.done().onTrue(new AimAndDriveCommand(swerve)
        .until(() -> swerve.isAimedAtHub())
        .andThen(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).alongWith(intakePivot.slamtake())
        ));


        returnToShoot.doneDelayed(0.5).onTrue(new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).alongWith(intakePivot.slamtake()));
 
        return routine;

    }

    
    private AutoRoutine C_BEELINE_GREED() {
        final AutoRoutine routine = autoFactory.newRoutine("C_BEELINE_GREED_ROUTINE");
        final AutoTrajectory goOverBumpTraj = C_BEELINE_FAMILY$0.asAutoTraj(routine);
        final AutoTrajectory getBallsInCenterTraj = C_BEELINE_FAMILY$1.asAutoTraj(routine);
        final AutoTrajectory returnToShoot = C_BEELINE_FAMILY$2.asAutoTraj(routine);
        final AutoTrajectory resetFromFinal = C_BEELINE_FAMILY$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
            )
        );

        goOverBumpTraj.done().onTrue(intakePivot.deployCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT));
        goOverBumpTraj.done().onTrue(getBallsInCenterTraj.cmd().alongWith(intakeRollers.intakeCommand()
        .alongWith(floor.setPercentOutCommand(Settings.IntakeSystemSettings.INTAKING_FLOOR_DUTYCYCLE)
        .alongWith(feeder.setPercentOutCommand(Settings.IntakeSystemSettings.INTAKING_FEEDER_DUTYCYCLE)))));


  
        getBallsInCenterTraj.done().onTrue(returnToShoot.cmd());
        returnToShoot.done().onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).alongWith(intakePivot.slamtake())
            .withTimeout(5)
            .andThen(resetFromFinal.cmd()
            .alongWith(intakePivot.confirmDeploy())));

        resetFromFinal.done().onTrue(
            goOverBumpTraj.cmd().andThen(getBallsInCenterTraj.cmd().alongWith(intakeRollers.intakeCommand()))
        );
        
        return routine;
    }

    private AutoRoutine C_PRELOAD() {
        final AutoRoutine routine = autoFactory.newRoutine("C_PRELOAD_ROUTINE");

        final AutoTrajectory lineUpToDepot = C_HOME_FAMILY$0.asAutoTraj(routine);

        routine.active().onTrue(
            lineUpToDepot.resetOdometry()
            .andThen(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem)
                .alongWith(intakePivot.slamtake())
                .withTimeout(6)
            )
        );
        return routine;
    }

    private AutoRoutine C_DEPOT() {
        final AutoRoutine routine = autoFactory.newRoutine("C_DEPOT_ROUTINE");
        final AutoTrajectory lineUpToDepot = C_HOME_FAMILY$0.asAutoTraj(routine);
        final AutoTrajectory getDepots = C_HOME_FAMILY$1.asAutoTraj(routine);
        final AutoTrajectory goToScoringPos = C_HOME_FAMILY$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                lineUpToDepot.resetOdometry(),
                lineUpToDepot.cmd().alongWith(intakePivot.timedDeployCommand())
            )
        );

        lineUpToDepot.done().onTrue(
            getDepots.cmd().alongWith(intakeRollers.intakeCommand())
        );

        getDepots.done().onTrue(goToScoringPos.cmd());

        goToScoringPos.done().onTrue(new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).alongWith(intakePivot.slamtake()));

        return routine;
    }

    private AutoRoutine D_PRELOAD() {
        final AutoRoutine routine = autoFactory.newRoutine("D_PRELOAD_ROUTINE");

        final AutoTrajectory lineUpToDepot = D_HOME_FAMILY$0.asAutoTraj(routine);

        routine.active().onTrue(
            lineUpToDepot.resetOdometry()
            .andThen(
                new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem)
                .alongWith(intakePivot.slamtake())
                .withTimeout(6)
            )
        );


        return routine;
    }

    private AutoRoutine D_DEPOT() {
        final AutoRoutine routine = autoFactory.newRoutine("C_DEPOT_ROUTINE");
        final AutoTrajectory lineUpToDepot = D_HOME_FAMILY$0.asAutoTraj(routine);
        final AutoTrajectory getDepots = D_HOME_FAMILY$1.asAutoTraj(routine);
        final AutoTrajectory goToScoringPos = D_HOME_FAMILY$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                lineUpToDepot.resetOdometry(),
                lineUpToDepot.cmd().alongWith(intakePivot.timedDeployCommand())
            )
        );

        lineUpToDepot.done().onTrue(
            getDepots.cmd().alongWith(intakeRollers.intakeCommand())
        );

        getDepots.done().onTrue(goToScoringPos.cmd());

        goToScoringPos.done().onTrue(new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem).alongWith(intakePivot.slamtake()));

        return routine;
    }



}