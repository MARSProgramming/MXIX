package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.constants.Settings;
import frc.robot.util.ShotSetup;
import frc.robot.util.ShotSetup.ShotInfo;

public class Superstructure extends SubsystemBase {
    
    private Cowl mCowl;
    private Swerve mSwerve;
    private Floor mFloor;
    private FastClimber mFastClimber;
    private Flywheel mFlywheel;
    private Feeder mFeeder;
    private IntakePivot mIntakePivot;
    private IntakeRollers mIntakeRollers;
    
    private ShotSetup shotInfoCalculator;

    private AimAndDriveCommand aimAndDriveCommand;

    public Superstructure(Cowl cowl, Swerve swerve, Feeder feeder, Floor floor, FastClimber fastClimber, Flywheel flywheel, IntakePivot intakePivot, IntakeRollers intakeRollers) {
        mCowl = cowl;
        mSwerve = swerve;
        mFloor = floor;
        mFeeder = feeder;
        mFlywheel = flywheel;
        mFastClimber = fastClimber;
        mIntakeRollers = intakeRollers;
        mIntakePivot = intakePivot;

        shotInfoCalculator = new ShotSetup();
    }


    public Command slamtake() {
    return Commands.repeatingSequence(
        mIntakePivot.deployCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT),
        Commands.waitSeconds(0.5),
        mIntakePivot.retractCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT),
        Commands.waitSeconds(0.5)
        );
    }

    public Command prepareStaticShot() {
        return Commands.parallel(
            mFlywheel.setVelocity(
                () -> shotInfoCalculator.getStaticShotInfo(mSwerve.getDistanceToHub()).shot.shooterRPM
            ),
            mCowl.setPositionCommand(
                () -> shotInfoCalculator.getStaticShotInfo(mSwerve.getDistanceToHub()).cowlPosition
            )
        );
    }

    public Command intakeCommand() {
        return Commands.parallel(
            mIntakeRollers.set(Settings.IntakeSystemSettings.INTAKING_STANDARD_DUTYCYCLE),
            mFloor.set(Settings.IntakeSystemSettings.INTAKING_FLOOR_STANDARD_DUTYCYCLE)
        );
    }

    public Command feedCommand() {
        return Commands.parallel(
            mFeeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE),
            mFloor.set(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE),
            mIntakeRollers.set(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE)
        );
    }

    public Command unjamCommand() {
        return Commands.parallel(
            mFeeder.setPercentOut(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE),
            mFloor.set(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE),
            mIntakeRollers.set(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE)
        );
    }

    public Command aimAndStaticShot(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
    AimAndDriveCommand aimAndDrive = new AimAndDriveCommand(mSwerve, forwardInput, leftInput);
    return Commands.parallel(
        aimAndDrive,
        prepareStaticShot(),
        new WaitUntilCommand(
            () -> mFlywheel.isVelocityWithinTolerance() && aimAndDrive.isAimed()
        ).andThen(feedCommand())
    );
}


}
