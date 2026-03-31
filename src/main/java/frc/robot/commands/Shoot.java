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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
/**
 * Command: Shoot
 * Executes the Shoot action.
 */
public class Shoot extends Command {
    private boolean readyToShootBoolean = false;

    private final Swerve swerve;
    private final Cowl cowl;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final LEDSubsystem ledsubsystem;


    private final ShotSetup shotSetup;
    private final DriveInputSmoother inputSmoother;

    // Request to drive field-centric while facing a specific angle
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Drive.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Drive.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);


    public Shoot(
        Swerve swerve,
        Cowl cowl,
        Flywheel flywheel,
        Feeder feeder,
        Floor floor,
        IntakeRollers intakeRollers,
        LEDSubsystem ledsubsystem,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.cowl = cowl;
        this.feeder = feeder;
        this.floor = floor;
        this.intakeRollers = intakeRollers;
        this.flywheel = flywheel;
        this.ledsubsystem = ledsubsystem;

        shotSetup = new ShotSetup();

        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, cowl, flywheel, feeder, floor, intakeRollers);
    }

    public Shoot(Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder, Floor floor, IntakeRollers intakeRollers, LEDSubsystem ledSubsystem) {
        this(swerve, cowl, flywheel, feeder, floor, intakeRollers,ledSubsystem, () -> 0, () -> 0);
    }



    @Override
    public void initialize() {
      readyToShootBoolean = false;
    }

    @Override
    public void execute() {
        // Get smoothed joystick inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        // Get shooting parameters

        ShotSetup.ShotInfo info = shotSetup.getStaticShotInfo(swerve.getDistanceToHub());

        double cowlAngle = info.cowlPosition;
        double shooterRPM = info.shot.shooterRPM;

        boolean cowlAtTolerance = cowl.isAtTolerance(cowlAngle);
        boolean flywheelAtTolerance = flywheel.isVelocityWithinTolerance(Units.RPM.of(shooterRPM));

        DogLog.log("AimAndShootCommand/flywheelReady", flywheelAtTolerance);
        DogLog.log("AimAndShootCommand/TargetVelocityRPM", shooterRPM);
        DogLog.log("AimAndShootCommand/cowlInTolerance", cowlAtTolerance);


        cowl.setPosition(cowlAngle);
        flywheel.setRPM(shooterRPM);

        if (cowlAtTolerance && flywheelAtTolerance) {
            readyToShootBoolean = true;
        }

        if (readyToShootBoolean) {
            feeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(cowl.home());
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        flywheel.setRPM(0);
        intakeRollers.setPercentOut(0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }

}