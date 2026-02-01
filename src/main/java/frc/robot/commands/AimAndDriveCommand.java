package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.constants.FieldConstants.Locations;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.GeometryUtil;
import frc.robot.util.ManualDriveInput;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
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
    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    /**
     * Constructs a new AimAndDriveCommand with zero translation (aim only).
     *
     * @param swerve The swerve subsystem.
     */
    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    /**
     * Checks if the robot is currently facing the target within tolerance.
     *
     * @return true if the robot's heading is close to the target heading.
     */
    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        
        // Get current heading in Blue Alliance perspective (standard field coordinates)
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        
        // Convert to Operator Perspective to match the request's frame of reference
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
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        
        // Calculate angle in standard field coordinates (Blue Alliance origin)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        
        // Adjust for the driver's perspective (Red vs Blue alliance)
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        
        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        // Get smoothed joystick inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        // Apply control request to swerve
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Drive.kMaxSpeed.times(input.forward))
                .withVelocityY(Drive.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub()) // Continuously update target direction
        );
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }
}