package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.constants.FieldConstants.Locations;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;
import frc.robot.util.ShuttleSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class AimAndShuttle extends Command {

    private final Swerve swerve;
    private final Cowl cowl;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final LEDSubsystem ledsubsystem;


    private final ShuttleSetup shuttleSetup;
    private final DriveInputSmoother inputSmoother;
    private Pose2d selectedShuttleTarget;

    // Request to drive field-centric while facing a specific angle
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Drive.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Drive.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);


    public AimAndShuttle(
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

        shuttleSetup = new ShuttleSetup();

        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, cowl, flywheel, feeder, floor, intakeRollers, ledsubsystem);
    }

    @Override
    public void initialize() {
        selectedShuttleTarget = FieldConstants.Locations.getClosestShuttlingPosition(swerve.getState().Pose);

    }

    @Override
    public void execute() {
        // Get smoothed joystick inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        // Get shooting parameters

        ShuttleSetup.ShuttleShotInfo info = shuttleSetup.getStaticShotInfo(swerve.getDistanceToHub());

        double cowlAngle = info.cowlPosition;
        double shooterRPM = info.shot.shooterRPM;

        // Apply control request to swerve
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Drive.kMaxSpeed.times(input.forward))
                .withVelocityY(Drive.kMaxSpeed.times(input.left))
                .withTargetDirection(swerve.getShooterDirectionToShuttle()) 
        );

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(shooterRPM);

        if (swerve.isAimedAtShuttle()) {
            DogLog.log("AimAndShuttleCommand/flywheelReady", flywheel.isVelocityWithinTolerance(Units.RPM.of(shooterRPM)));
            DogLog.log("AimAndShuttleCommand/TargetVelocityRPM", shooterRPM);
            DogLog.log("AimAndShuttleCommand/cowlInTolerance", cowl.isAtTolerance(cowlAngle));


            feeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE);

            ledsubsystem.strobe(Color.kBlue, LEDSegment.ALL);
        }
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(cowl.home());
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        flywheel.setRPM(0);
        intakeRollers.setPercentOut(0);

        if (interrupted) {
            ledsubsystem.rainbow(LEDSegment.ALL);
        }
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }

}