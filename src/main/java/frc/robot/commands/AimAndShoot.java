package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

public class AimAndShoot extends Command {

    private boolean mReadyToShoot  = false;
    private boolean mCowlReady     = false;
    private boolean mFlywheelReady = false;

    private final Swerve          swerve;
    private final Cowl            cowl;
    private final Flywheel        flywheel;
    private final Feeder          feeder;
    private final Floor           floor;
    private final IntakeRollers   intakeRollers;
    private final ShotSetup       shotSetup;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle facingAngleRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Drive.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Drive.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0);

    public AimAndShoot(
        Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder,
        Floor floor, IntakeRollers intakeRollers, ShotSetup shotSetup,
        DoubleSupplier forwardInput, DoubleSupplier leftInput
    ) {
        this.swerve        = swerve;
        this.cowl          = cowl;
        this.flywheel      = flywheel;
        this.feeder        = feeder;
        this.floor         = floor;
        this.intakeRollers = intakeRollers;
        this.shotSetup     = shotSetup;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, cowl, flywheel, feeder, floor, intakeRollers);
    }

        /**
     * Constructs a new Shooting command with zero translation (aim only).
     *
     * @param swerve The swerve subsystem.
     */
    public AimAndShoot(Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder, Floor floor, IntakeRollers intakeRollers, ShotSetup shotSetup) {
        this(swerve, cowl, flywheel, feeder, floor, intakeRollers, shotSetup, () -> 0, () -> 0);
    }

    @Override
    public void initialize() {
        mReadyToShoot  = false;
        mCowlReady     = false;
        mFlywheelReady = false;
    }

    @Override
    public void execute() {
        ManualDriveInput input   = inputSmoother.getSmoothedInput();
        boolean          isAimed = swerve.isAimedAtHub();

        swerve.setControl(facingAngleRequest
            .withVelocityX(Drive.kMaxSpeed.times(input.forward))
            .withVelocityY(Drive.kMaxSpeed.times(input.left))
            .withTargetDirection(swerve.getShooterDirectionToHub()));

        ShotSetup.ShotInfo info      = shotSetup.getStaticShotInfo(swerve.getDistanceToHub());
        double             cowlAngle = info.cowlPosition;
        double             rpm       = info.shot.shooterRPM;

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(rpm);

        mCowlReady     = cowl.isAtTolerance(cowlAngle);
        mFlywheelReady = flywheel.isVelocityWithinTolerance(Units.RPM.of(rpm));

        DogLog.log("AimAndShoot/CowlTarget",      cowlAngle);
        DogLog.log("AimAndShoot/RPMTarget",        rpm);
        DogLog.log("AimAndShoot/CowlReady",        mCowlReady);
        DogLog.log("AimAndShoot/FlywheelReady",    mFlywheelReady);
        DogLog.log("AimAndShoot/Aimed",            isAimed);

        if (mCowlReady && mFlywheelReady && isAimed) mReadyToShoot = true;

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