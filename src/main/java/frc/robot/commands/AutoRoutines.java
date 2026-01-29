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
* Handles autonomous routine selection and configuration
**/ 
public final class AutoRoutines {
    private final Swerve swerve;
    private final Limelight shooterLimelight;
    private final Limelight backLimelight;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

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

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::OutpostRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine OutpostRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost");
        final AutoTrajectory startToShoot = OutpostTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory shootAndMovetoPreintake = OutpostTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory intakeThenMoveToShoot = OutpostTrajectory$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                startToShoot.resetOdometry(),
                startToShoot.cmd()
            )
        );

        startToShoot.active().whileTrue(shooterLimelight.idle().alongWith(backLimelight.idle())); // Keep Limelights idle while driving. useful for paths that rotate or move fast.
        startToShoot.done().onTrue(shootAndMovetoPreintake.cmd());
        shootAndMovetoPreintake.done().onTrue(intakeThenMoveToShoot.cmd());

        return routine;
    }
}