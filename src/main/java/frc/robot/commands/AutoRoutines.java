// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.ChoreoTraj.OutpostTrajectory$0;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$1;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$2;
import static frc.robot.util.ChoreoTraj.ResetPoseOutpost;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;

/**
 * Handles autonomous routine selection and configuration using Choreo.
 * This class manages the creation of auto routines and publishes the selector to the dashboard.
 */
public final class AutoRoutines {
    private final Superstructure mSuperstructure;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    /**
     * Creates a new AutoRoutines manager.
     *
     * @param superstructure The integrated robot subsystem.
     */
    public AutoRoutines(
        Superstructure superstructure
    ) {

        mSuperstructure = superstructure;

        this.autoFactory = mSuperstructure.getSwerveSubsystem().createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("Outpost", this::OutpostRoutine);
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
        startToShoot.active().whileTrue(mSuperstructure.idleBothCameras()); 
        
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
}