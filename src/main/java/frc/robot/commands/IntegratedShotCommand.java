package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShotSetup;

public class IntegratedShotCommand extends ParallelCommandGroup {
        private ShotSetup mSetup;
        private Swerve mSwerve;
        private Flywheel mFlywheel;
        private Cowl mCowl;
        private IntakeRollers mIntakeRollers;
        private Feeder mFeeder;
        private Floor mFloor;

        DoubleSupplier forwardInput;
        DoubleSupplier leftInput;

        public IntegratedShotCommand(DoubleSupplier forw, DoubleSupplier lef,  ShotSetup setup, Swerve swerve, Flywheel flywheel, Cowl cowl, IntakeRollers rollers, Feeder feeder, Floor floor) {
            mSwerve = swerve;
            mSetup = setup;
            mFlywheel = flywheel;
            mCowl = cowl;
            mIntakeRollers = rollers;
            mFeeder = feeder;
            mFloor = floor;

            forwardInput = forw;
            leftInput = lef;

        PrepareSupershot supershot = new PrepareSupershot(setup, swerve, flywheel, cowl, forwardInput, leftInput);
        FeedCommand feed = new FeedCommand(rollers, feeder, floor);

        addCommands(
        supershot,
        new WaitUntilCommand(() -> supershot.readyToShoot())
        .andThen(feed)
        );
    }   
}
