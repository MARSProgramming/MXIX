// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.ChoreoTraj.OutpostTrajectory$0;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$1;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$2;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * Handles autonomous routine selection and configuration using Choreo.
 * This class manages the creation of auto routines and publishes the selector to the dashboard.
 */
public final class AutoRoutines {
    private final Swerve swerve;
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
        Limelight shooterLimelight,
        Limelight backLimelight
    ) {
        this.swerve = swerve;
        this.shooterLimelight = shooterLimelight;
        this.backLimelight = backLimelight;
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("Outpost", this::OutpostRoutine);
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
        startToShoot.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle())); 
        
        // Chain the trajectories: when one finishes, start the next
        startToShoot.done().onTrue(shootAndMovetoPreintake.cmd());
        shootAndMovetoPreintake.done().onTrue(intakeThenMoveToShoot.cmd());

        return routine;
    }
}