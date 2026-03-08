// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.ChoreoTraj.OutpostTrajectory$0;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$1;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$2;
import static frc.robot.util.ChoreoTraj.ResetPoseOutpost;
import static frc.robot.util.ChoreoTraj.ShootPreloaded;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.FastClimber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
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
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("driveToPreloaded pos", this::drivetoPreloadShootPos);
        autoChooser.addRoutine("reset Pose", this::resetPoseAtTrench);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Schedule the selected autonomous command when the robot enters autonomous mode
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    /**
     * Defines the "Outpost" autonomous routine.
     * This routine follows a sequence of trajectories defined in Choreo.
     *
     * @return The configured AutoRoutine.
     */
    private AutoRoutine OutpostRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost");
        
        // Load trajectories from the generated ChoreoTraj constants
        final AutoTrajectory startToShoot = OutpostTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory shootAndMovetoPreintake = OutpostTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory intakeThenMoveToShoot = OutpostTrajectory$2.asAutoTraj(routine);

        // Define the sequence of events
        routine.active().onTrue(
            Commands.sequence(
                startToShoot.resetOdometry(), // Reset pose to start of path
                startToShoot.cmd()
            )
        );

        // Keep Limelights idle while driving the first path. Useful for paths that rotate or move fast.
        startToShoot.active().whileTrue(shooterLimelight.idle()); 
        
        // Chain the trajectories: when one finishes, start the next
        startToShoot.done().onTrue(shootAndMovetoPreintake.cmd());
        shootAndMovetoPreintake.done().onTrue(intakeThenMoveToShoot.cmd());

        return routine;
    }

    private AutoRoutine resetPoseAtTrench() {
        final AutoRoutine routine = autoFactory.newRoutine("reset Pose at trench");
        final AutoTrajectory posTrajectory = ResetPoseOutpost.asAutoTraj(routine);

        routine.active().onTrue(
            posTrajectory.resetOdometry()
        );

        return routine;
    }  
    
    private AutoRoutine drivetoPreloadShootPos() {
        final AutoRoutine routine = autoFactory.newRoutine("Drive to preload shot");
        final AutoTrajectory posTrajectory = ShootPreloaded.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                posTrajectory.resetOdometry(), // Reset pose to start of path
                posTrajectory.cmd()
            )
        );

        posTrajectory.done().onTrue(new AimAndShoot(swerve, cowl, flywheel, feeder, floor, intakeRollers).withTimeout(5));

        return routine;
    }
}