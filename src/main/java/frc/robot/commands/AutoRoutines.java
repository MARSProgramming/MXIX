// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.ChoreoTraj.OutpostTrajectory$0;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$1;
import static frc.robot.util.ChoreoTraj.OutpostTrajectory$2;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddle;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddle$0;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddle$1;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddleOutpost;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddleOutpost$0;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddleOutpost$1;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddleOutpost$2;
import static frc.robot.util.ChoreoTraj.RightSideBumpMiddleOutpost$3;
import static frc.robot.util.ChoreoTraj.RightSideBumpFullPass;
import static frc.robot.util.ChoreoTraj.RightSideBumpFullPass$0;
import static frc.robot.util.ChoreoTraj.RightSideBumpFullPass$1;
import static frc.robot.util.ChoreoTraj.RightSideBumpFullPass$2;
import static frc.robot.util.ChoreoTraj.RightSideBumpFullPass$3;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddle;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddle$0;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddle$1;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddleDepot;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddleDepot$0;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddleDepot$1;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddleDepot$2;
import static frc.robot.util.ChoreoTraj.LeftSideBumpMiddleDepot$3;
import static frc.robot.util.ChoreoTraj.LeftSideBumpFullPass;
import static frc.robot.util.ChoreoTraj.LeftSideBumpFullPass$0;
import static frc.robot.util.ChoreoTraj.LeftSideBumpFullPass$1;
import static frc.robot.util.ChoreoTraj.LeftSideBumpFullPass$2;
import static frc.robot.util.ChoreoTraj.LeftSideBumpFullPass$3;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * Handles autonomous routine selection and configuration using Choreo.
 * This class manages the creation of auto routines and publishes the selector
 * to the dashboard.
 */
public final class AutoRoutines {
    private final Swerve swerve;
    private final Limelight shooterLimelight;
    private final Limelight backLimelight;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    // Event map for Choreo event markers.
    Map<String, Command> eventMap = Map.of(
            "Intake", Commands.print("Intake Marker Triggered"),
            "Shoot", Commands.print("Shoot Marker Triggered"));

    /**
     * Creates a new AutoRoutines manager.
     *
     * @param swerve           The swerve subsystem used for path following.
     * @param shooterLimelight The limelight used for shooter aiming.
     * @param backLimelight    The back limelight.
     */
    public AutoRoutines(
            Swerve swerve,
            Limelight shooterLimelight,
            Limelight backLimelight) {
        this.swerve = swerve;
        this.shooterLimelight = shooterLimelight;
        this.backLimelight = backLimelight;
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configures the autonomous routines and publishes the chooser to
     * SmartDashboard.
     * Binds the selected routine to run when autonomous mode is enabled.
     */
    public void configure() {
        autoChooser.addRoutine("Outpost", this::OutpostRoutine);
        autoChooser.addRoutine("Right Side Bump Middle", this::rightSideBumpMiddle);
        autoChooser.addRoutine("Right Side Bump Middle Outpost", this::rightSideBumpMiddleOutpost);
        autoChooser.addRoutine("Right Side Bump Full Pass", this::rightSideBumpFullPass);
        autoChooser.addRoutine("Left Side Bump Middle", this::leftSideBumpMiddle);
        autoChooser.addRoutine("Left Side Bump Middle Depot", this::leftSideBumpMiddleDepot);
        autoChooser.addRoutine("Left Side Bump Full Pass", this::leftSideBumpFullPass);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Schedule the selected autonomous command when the robot enters autonomous
        // mode
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
                        startToShoot.cmd()));

        // Keep Limelights idle while driving the first path. Useful for paths that
        // rotate or move fast.
        startToShoot.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));

        // Chain the trajectories: when one finishes, start the next
        startToShoot.done().onTrue(shootAndMovetoPreintake.cmd());
        shootAndMovetoPreintake.done().onTrue(intakeThenMoveToShoot.cmd());

        return routine;
    }

    private AutoRoutine rightSideBumpMiddle() {
        final AutoRoutine routine = autoFactory.newRoutine("Right Side Bump Middle");
        final AutoTrajectory overTheBump = RightSideBumpMiddle$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = RightSideBumpMiddle$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());

        return routine;
    }

    private AutoRoutine rightSideBumpMiddleOutpost() {
        final AutoRoutine routine = autoFactory.newRoutine("Right Side Bump Middle Outpost");
        final AutoTrajectory overTheBump = RightSideBumpMiddleOutpost$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = RightSideBumpMiddleOutpost$1.asAutoTraj(routine);
        final AutoTrajectory backOverTheBump = RightSideBumpMiddleOutpost$2.asAutoTraj(routine);
        final AutoTrajectory intakeBallsOutpost = RightSideBumpMiddleOutpost$3.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());
        intakeBallsCenter.done().onTrue(backOverTheBump.cmd());
        backOverTheBump.done().onTrue(intakeBallsOutpost.cmd());

        return routine;
    }

        private AutoRoutine rightSideBumpFullPass() {
        final AutoRoutine routine = autoFactory.newRoutine("Right Side Bump Full Pass");
        final AutoTrajectory overTheBump = RightSideBumpFullPass$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = RightSideBumpFullPass$1.asAutoTraj(routine);
        final AutoTrajectory backOverTheBump = RightSideBumpFullPass$2.asAutoTraj(routine);
        final AutoTrajectory climb = RightSideBumpFullPass$3.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());
        intakeBallsCenter.done().onTrue(backOverTheBump.cmd());
        backOverTheBump.done().onTrue(climb.cmd());

        return routine;
    }

    private AutoRoutine leftSideBumpMiddle() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Side Bump Middle");
        final AutoTrajectory overTheBump = LeftSideBumpMiddle$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = LeftSideBumpMiddle$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());

        return routine;
    }

    private AutoRoutine leftSideBumpMiddleDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Side Bump Middle Depot");
        final AutoTrajectory overTheBump = LeftSideBumpMiddleDepot$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = LeftSideBumpMiddleDepot$1.asAutoTraj(routine);
        final AutoTrajectory backOverTheBump = LeftSideBumpMiddleDepot$2.asAutoTraj(routine);
        final AutoTrajectory intakeBallsOutpost = LeftSideBumpMiddleDepot$3.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());
        intakeBallsCenter.done().onTrue(backOverTheBump.cmd());
        backOverTheBump.done().onTrue(intakeBallsOutpost.cmd());

        return routine;
    }

    private AutoRoutine leftSideBumpFullPass() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Side Bump Full Pass");
        final AutoTrajectory overTheBump = LeftSideBumpFullPass$0.asAutoTraj(routine);
        final AutoTrajectory intakeBallsCenter = LeftSideBumpFullPass$1.asAutoTraj(routine);
        final AutoTrajectory backOverTheBump = LeftSideBumpFullPass$2.asAutoTraj(routine);
        final AutoTrajectory climb = LeftSideBumpFullPass$3.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        overTheBump.resetOdometry(), // Reset pose to start of path
                        overTheBump.cmd()));

        overTheBump.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle()));
        overTheBump.done().onTrue(intakeBallsCenter.cmd());
        intakeBallsCenter.done().onTrue(backOverTheBump.cmd());
        backOverTheBump.done().onTrue(climb.cmd());

        return routine;
    }
}