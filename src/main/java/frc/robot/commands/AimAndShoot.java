package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.constants.FieldConstants.Locations;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.GeometryUtil;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class AimAndShoot extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);
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
    public AimAndShoot(
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
     * Constructs a new AimAndDriveCommand with zero translation (aim only).
     *
     * @param swerve The swerve subsystem.
     */
    public AimAndShoot(Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder, Floor floor, IntakeRollers intakeRollers) {
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
        
        // Get shooting parameters
        final double swerveDistanceToHub = swerve.getDistanceToHub();

        double cowlAngle = shotSetup.getStaticShotInfo(swerveDistanceToHub).cowlPosition;
        double shooterRPM = shotSetup.getStaticShotInfo(swerveDistanceToHub).shot.shooterRPM;

        // Apply control request to swerve
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Drive.kMaxSpeed.times(input.forward))
                .withVelocityY(Drive.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub()) 
        );

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(shooterRPM);

        if (isAimed() && flywheel.isVelocityWithinTolerance()) {
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
        cowl.home();
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

        /**
     * Checks if the robot is currently facing the target within tolerance.
     *
     * @return true if the robot's heading is close to the target heading.
     */
    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    /**
     * Calculates the direction from the robot to the Hub (target).
     *
     * @return The Rotation2d representing the angle to the target in Operator Perspective.
     */
    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Locations.hubPosition();
        final Pose2d robotPose = swerve.getState().Pose;
        Pose2d launcherPosition = robotPose.transformBy(GeometryUtil.toTransform2d(SystemConstants.Flywheel.ROBOT_TO_SHOOTER_TRANSFORM));
        final Translation2d shooterPos = launcherPosition.getTranslation();

        
        // Calculate angle in standard field coordinates (Blue Alliance origin)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(shooterPos).getAngle();
        
        // Adjust for the driver's perspective (Red vs Blue alliance)
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        
        return hubDirectionInOperatorPerspective;
    }

}