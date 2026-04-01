// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.util.ChoreoTraj.B_BEELINE_OLD$0;
import static frc.robot.util.ChoreoTraj.B_BEELINE_OLD$1;
import static frc.robot.util.ChoreoTraj.B_BEELINE_OLD$2;
import static frc.robot.util.ChoreoTraj.B_BEELINE_OLD$3;
import static frc.robot.util.ChoreoTraj.B_BEELINE_OLD$4;
import static frc.robot.util.ChoreoTraj.C_BEELINE_OLD$0;
import static frc.robot.util.ChoreoTraj.C_BEELINE_OLD$1;
import static frc.robot.util.ChoreoTraj.C_BEELINE_OLD$2;
import static frc.robot.util.ChoreoTraj.C_BEELINE_OLD$3;
import static frc.robot.util.ChoreoTraj.C_BEELINE_OLD$4;
import static frc.robot.util.ChoreoTraj.C_CLIMB_NEARD$0;
import static frc.robot.util.ChoreoTraj.C_CLIMB_NEARD$1;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$0;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$1;
import static frc.robot.util.ChoreoTraj.C_DEPOT_AND_CLIMB$2;
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
import frc.robot.util.ShotSetup;
import frc.robot.subsystems.Swerve;

/**
 * Handles autonomous routine selection and configuration using Choreo.
 *
 * <p>Path Naming Convention:
 * <ul>
 *   <li>B = OutpostBump (Red alliance starting position near outpost)</li>
 *   <li>C = DepotBump (Blue alliance starting position near depot)</li>
 *   <li>X = Middle (Center starting position)</li>
 *   <li>BEELINE = Direct path over terrain bump to scoring position</li>
 *   <li>CLIMB = Path to climbing position</li>
 *   <li>DEPOT_AND_CLIMB = Score at depot then climb</li>
 *   <li>NEARD = Near depot (climbing position)</li>
 * </ul>
 *
 * <p>Example: C_DEPOT_AND_CLIMB = Blue alliance (DepotBump) path that scores at depot and then climbs
 *
 * <p>This class manages the creation of auto routines and publishes the selector to the dashboard.
 */
public final class AutoRoutines {

    /**
     * Starting positions for autonomous modes.
     * These correspond to the robot's initial position on the field for each alliance.
     */
    public enum StartingPosition {
        /** Center field start position - X paths */
        MIDDLE("Middle", "X"),

        /** Blue alliance depot side start position - C paths */
        BLUE_DEPOT_BUMP("DepotBump", "C"),

        /** Red alliance outpost side start position - B paths */
        RED_OUTPOST_BUMP("OutpostBump", "B");

        private final String dashboardName;
        private final String pathPrefix;

        StartingPosition(String dashboardName, String pathPrefix) {
            this.dashboardName = dashboardName;
            this.pathPrefix = pathPrefix;
        }

        /** Gets the display name shown on the dashboard */
        public String getDashboardName() {
            return dashboardName;
        }

        /** Gets the path prefix used in Choreo trajectory names */
        public String getPathPrefix() {
            return pathPrefix;
        }
    }

    /**
     * Auto routine types describing the main action of the path.
     */
    public enum AutoRoutineType {
        /** Direct path over bump to scoring position */
        BEELINE("Score Beeline"),

        /** Score preloaded game piece then climb */
        SCORE_PRELOAD_CLIMB("Score Preload Climb"),

        /** Score at depot then climb */
        SCORE_DEPOT_CLIMB("Score Depot Climb"),

        /** Reset odometry without movement */
        RESET_ODOM("Reset Odom");

        private final String dashboardName;

        AutoRoutineType(String dashboardName) {
            this.dashboardName = dashboardName;
        }

        public String getDashboardName() {
            return dashboardName;
        }
    }
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
     *
     * <p>Available Auto Modes:
     * <ul>
     *   <li>DepotBump (Blue Alliance) - Start near depot side</li>
     *   <li>Middle (Center) - Start at center field</li>
     *   <li>OutpostBump (Red Alliance) - Start near outpost side</li>
     * </ul>
     */
    public void configure() {
        // Blue Alliance (DepotBump) routines
        autoChooser.addRoutine("DepotBump Score Beeline", this::CBeeline);
        autoChooser.addRoutine("DepotBump Score Preload Climb", this::CClimbNearDepot);
        autoChooser.addRoutine("DepotBump Score Depot Climb", this::CScoreDepotClimb);

        // Center (Middle) routines
        autoChooser.addRoutine("Middle Score Preload Climb", this::XClimbNearDepot);
        autoChooser.addRoutine("Middle Score Depot Climb", this::XScoreDepotClimb);
        autoChooser.addRoutine("Middle Reset Odom", this::XClimbResetOdom);

        // Red Alliance (OutpostBump) routines
        autoChooser.addRoutine("OutpostBump Score Beeline", this::BBeeline);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Schedule the selected autonomous command when the robot enters autonomous mode
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler()
        .beforeStarting((() -> ledsubsystem.setColor(Color.kRed, LEDSegment.ALL)))
        .finallyDo(() -> ledsubsystem.rainbow(LEDSegment.ALL)));
    }





    
    /**
     * Blue Alliance (DepotBump) Beeline Routine.
     * Scores preloaded game piece, then crosses bump and collects game pieces.
     * Returns to scoring position for final shot.
     */
    private AutoRoutine CBeeline() {
        final AutoRoutine routine = autoFactory.newRoutine("BLUE_DEPOT_BUMP_BEELINE");
        final AutoTrajectory goOverBumpTraj = C_BEELINE_OLD$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = C_BEELINE_OLD$1.asAutoTraj(routine);
        final AutoTrajectory sweepBallsTraj = C_BEELINE_OLD$2.asAutoTraj(routine);
        final AutoTrajectory getBackOver = C_BEELINE_OLD$3.asAutoTraj(routine);
        final AutoTrajectory finalTurnTraj = C_BEELINE_OLD$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
                .beforeStarting(() -> ledsubsystem.rainbow(LEDSegment.ALL))
            )
        );

        goOverBumpTraj.done().onTrue(prepSweep.cmd().alongWith(intakePivot.timedDeployCommand()));
        prepSweep.doneDelayed(0.25).onTrue(sweepBallsTraj.cmd().alongWith(intakeRollers.intakeCommand()));

        sweepBallsTraj.done().onTrue(getBackOver.cmd());

        getBackOver.done().onTrue(finalTurnTraj.cmd());

        finalTurnTraj.doneDelayed(1.0).onTrue(
            new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup) 
            .alongWith(intakePivot.slamtake())
        );

        return routine;
    }

    


    
    /**
     * Red Alliance (OutpostBump) Beeline Routine.
     * Mirror of blue alliance beeline - scores preloaded game piece,
     * crosses bump to collect game pieces, returns for final shot.
     */
    private AutoRoutine BBeeline() {
        final AutoRoutine routine = autoFactory.newRoutine("RED_OUTPOST_BUMP_BEELINE");
        final AutoTrajectory goOverBumpTraj = B_BEELINE_OLD$0.asAutoTraj(routine);
        final AutoTrajectory prepSweep = B_BEELINE_OLD$1.asAutoTraj(routine);
        final AutoTrajectory sweepBallsTraj = B_BEELINE_OLD$2.asAutoTraj(routine);
        final AutoTrajectory getBackOver = B_BEELINE_OLD$3.asAutoTraj(routine);
        final AutoTrajectory finalTurnTraj = B_BEELINE_OLD$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                goOverBumpTraj.resetOdometry(),
                goOverBumpTraj.cmd()
                .beforeStarting(() -> ledsubsystem.rainbow(LEDSegment.ALL))
            )
        );

        goOverBumpTraj.done().onTrue(prepSweep.cmd().alongWith(intakePivot.timedDeployCommand()));
        prepSweep.doneDelayed(0.25).onTrue(sweepBallsTraj.cmd().alongWith(intakeRollers.intakeCommand()));

        sweepBallsTraj.done().onTrue(getBackOver.cmd());


        getBackOver.done().onTrue(finalTurnTraj.cmd());

        finalTurnTraj.doneDelayed(1.0).onTrue(
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
            .andThen(
                fastClimber.setPercentOut(Settings.ClimbSettings.CLIMB_DUTYCYCLE).withTimeout(5)
            )
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