package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        this.mCowl = cowlSupplier;
        this.mFlipClimber = flipClimbSupplier;
        this.mFastClimber = fastClimbSupplier;
        this.mIntakePivot = intakePivotSupplier;
        this.mIntakeRollers = intakeRollersSupplier;
        this.mFlywheel = flywheelSupplier;
        this.mFloor = floorSupplier;
        this.mFeeder = feederSupplier;

    }

    

}
