package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShotSetup;

/**
 * Runs shooter systems (cowl, flywheel, feeder) using SOTM compensation
 * while an external command (e.g. PathPlanner) controls the drivetrain.
 *
 * Does NOT require swerve — safe to run alongside any path-following command.
 */
public class ShootWhileFollowing extends Command {

    private boolean mReadyToShoot = false;

    private final Swerve        swerve;
    private final Cowl          cowl;
    private final Flywheel      flywheel;
    private final Feeder        feeder;
    private final Floor         floor;
    private final IntakeRollers intakeRollers;
    private final ShotSetup     shotSetup;

    public ShootWhileFollowing(
        Swerve swerve,
        Cowl cowl,
        Flywheel flywheel,
        Feeder feeder,
        Floor floor,
        IntakeRollers intakeRollers,
        ShotSetup shotSetup
    ) {
        this.swerve        = swerve;
        this.cowl          = cowl;
        this.flywheel      = flywheel;
        this.feeder        = feeder;
        this.floor         = floor;
        this.intakeRollers = intakeRollers;
        this.shotSetup     = shotSetup;

        // Intentionally NO swerve requirement — path command owns swerve
        addRequirements(cowl, flywheel, feeder, floor, intakeRollers);
    }

    @Override
    public void initialize() {
        mReadyToShoot = false;
    }

    @Override
    public void execute() {
        ShotSetup.SOTMInfo sotm     = shotSetup.getSOTMInfoHub(swerve);
        double             cowlAngle = sotm.shotInfo.cowlPosition;
        double             rpm       = sotm.shotInfo.shot.shooterRPM;

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(rpm);

        boolean spinning = flywheel.isVelocityWithinTolerance(RPM.of(rpm));

        DogLog.log("ShootWhileFollowing/CowlTarget",    cowlAngle);
        DogLog.log("ShootWhileFollowing/RPMTarget",      rpm);
        DogLog.log("ShootWhileFollowing/FlywheelReady",  spinning);
        DogLog.log("ShootWhileFollowing/ReadyToShoot",   mReadyToShoot);

        if (spinning) mReadyToShoot = true;

        if (mReadyToShoot) {
            feeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        flywheel.setRPM(0);
        intakeRollers.setPercentOut(0);
        cowl.setZeroOut();
    }

    @Override
    public boolean isFinished() { return false; }
}