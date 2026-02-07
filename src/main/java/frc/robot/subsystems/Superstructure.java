package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AimAndDriveCommand;

public class Superstructure extends SubsystemBase {
    Cowl mCowl;
    FlipClimber mFlipClimber;
    FastClimber mFastClimber;
    IntakePivot mIntakePivot;
    IntakeRollers mIntakeRollers;
    Flywheel mFlywheel;
    Floor mFloor;
    Feeder mFeeder;
    Swerve mSwerve;

    private AimAndDriveCommand aimAndDriveCommand;

    public Superstructure (
        Cowl cowlSupplier, 
        FlipClimber flipClimbSupplier, 
        FastClimber fastClimbSupplier, 
        IntakePivot intakePivotSupplier, 
        IntakeRollers intakeRollersSupplier, 
        Flywheel flywheelSupplier, 
        Floor floorSupplier, 
        Feeder feederSupplier, 
        Swerve swerveSupplier)  {

        // Subsystem initialization
        this.mCowl = cowlSupplier;
        this.mFlipClimber = flipClimbSupplier;
        this.mFastClimber = fastClimbSupplier;
        this.mIntakePivot = intakePivotSupplier;
        this.mIntakeRollers = intakeRollersSupplier;
        this.mFlywheel = flywheelSupplier;
        this.mFloor = floorSupplier;
        this.mFeeder = feederSupplier;    
        this.mSwerve = swerveSupplier;    

        // Command initialization
        aimAndDriveCommand = new AimAndDriveCommand(mSwerve);
    }

    public Command hubBaseShot() {
        return Commands.parallel(
            aimAndDriveCommand,
            mFlywheel.spinUp(3000),
            mFeeder.setVelocity(1000).onlyIf(() -> mFlywheel.isVelocityWithinTolerance() && aimAndDriveCommand.isAimed()),
            mFloor.set(0.5).onlyIf(() -> mFeeder.isVelocityWithinTolerance() && aimAndDriveCommand.isAimed())
        ).handleInterrupt(
            () -> {
                mFlywheel.setPercentOut(0);
                mFloor.set(0);
                mFeeder.setVelocity(0);
            }
        );
    }

    public Command intake() {
        return Commands.parallel(
            mIntakePivot.setPosition(1.0),
            mIntakeRollers.set(0.5)
        ).handleInterrupt(
            () -> {
                mIntakeRollers.set(0);
            }
        );
    }

    public Command TimedHubBaseShot(double timeoutSeconds) {
        return hubBaseShot().withTimeout(timeoutSeconds);
    }
}
