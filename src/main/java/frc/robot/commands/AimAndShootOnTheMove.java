package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class AimAndShootOnTheMove extends Command {
    private boolean readyToShootBoolean = false;

    private final Swerve swerve;
    private final Cowl cowl;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final IntakeRollers intakeRollers;
    private final Floor floor;


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

    /**
     * Constructs a new AimAndDriveCommand.
     *
     * @param swerve The swerve subsystem.
     * @param forwardInput Supplier for forward/backward translation input (-1 to 1).
     * @param leftInput Supplier for left/right translation input (-1 to 1).
     */
    public AimAndShootOnTheMove(
        Swerve swerve,
        Cowl cowl,
        Flywheel flywheel,
        Feeder feeder,
        Floor floor,
        IntakeRollers intakeRollers,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.cowl = cowl;
        this.feeder = feeder;
        this.floor = floor;
        this.intakeRollers = intakeRollers;
        this.flywheel = flywheel;

        shotSetup = new ShotSetup();

        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, cowl, flywheel, feeder, floor, intakeRollers);
    }

    /**
     * Constructs a new Shooting command with zero translation (aim only).
     *
     * @param swerve The swerve subsystem.
     */
    public AimAndShootOnTheMove(Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder, Floor floor, IntakeRollers intakeRollers) {
        this(swerve, cowl, flywheel, feeder, floor, intakeRollers, () -> 0, () -> 0);
    }



    @Override
    public void initialize() {
      readyToShootBoolean = false;
    }

    @Override
    public void execute() {
        // Get smoothed joystick inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        ShotSetup.SOTMInfo sotmInfo = shotSetup.getSOTMInfoHub(swerve);

        double cowlAngle = sotmInfo.shotInfo.cowlPosition;
        double shooterRPM = sotmInfo.shotInfo.shot.shooterRPM;
        Rotation2d virtualTargetAngle = sotmInfo.virtualTargetAngle; 
        AngularVelocity targetRateFeedforward = Units.RadiansPerSecond.of(sotmInfo.angularVelocityRadPerSec);       

        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Drive.kMaxSpeed.times(input.forward))
                .withVelocityY(Drive.kMaxSpeed.times(input.left))
                .withTargetDirection(virtualTargetAngle) 
                .withTargetRateFeedforward(targetRateFeedforward)
        );

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(shooterRPM);

        if (swerve.isAimedAtVirtualTarget(virtualTargetAngle) && flywheel.isVelocityWithinTolerance(RPM.of(shooterRPM))) {
            readyToShootBoolean = true;
        }

        if (readyToShootBoolean) {
            feeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE);
        }

        // Todo: can we just completely disable the feeding system when we aren't aimed while SOTM? or just assume
        // we are already ready? for now, we assume, but it would be useful for actual input confirmation.
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        flywheel.setRPM(0);
        intakeRollers.setPercentOut(0);
        CommandScheduler.getInstance().schedule(cowl.home());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}