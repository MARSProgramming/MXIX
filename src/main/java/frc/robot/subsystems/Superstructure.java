package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    Cowl mCowl;
    Flywheel mFlywheel;
    Feeder mFeeder;
    Floor mFloor;

   // private AimAndDriveCommand aimAndDriveCommand;

    public Superstructure (
        Cowl cowlSupplier, 
        Flywheel flywheelSupplier, 
        Feeder feederSupplier,
        Floor floorSupplier)  {

        // Subsystem initialization
        this.mCowl = cowlSupplier;
        this.mFlywheel = flywheelSupplier;
        this.mFeeder = feederSupplier;   
        this.mFloor = floorSupplier; 

        // Command initialization
       // aimAndDriveCommand = new AimAndDriveCommand(mSwerve);
    }

    public Command feedAndRunFlywheel() {
        return Commands.parallel(
            mFlywheel.spinUp(3000),
            mFeeder.setPercentOut(0.5).onlyIf(() -> mFlywheel.isVelocityWithinTolerance())

        ).handleInterrupt(
            () -> {
                mFlywheel.setPercentOut(0);
                mFeeder.setPercentOut(0);
            }
        );
    }

    /*
     
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
* 
     */

    

    /*
    public Command TimedHubBaseShot(double timeoutSeconds) {
        return hubBaseShot().withTimeout(timeoutSeconds);
    }

         */


}
