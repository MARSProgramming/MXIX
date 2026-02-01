package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SystemConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.Stopwatch;

/**
 * Teleop manual drive command with features for locked-heading driving.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 * 
 * The application of locking the heading will allow us to force the robot into a diagonal orientation for bump traversal.
 */

public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final Time kHeadingLockDelay = Seconds.of(0.25); // time to wait before locking heading

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    // The default teleoperated drive request.
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    // A request that handles field-centric driving with a locked heading.
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(SystemConstants.Drive.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(SystemConstants.Drive.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    private Stopwatch headingLockStopwatch = new Stopwatch();
    private ManualDriveInput previousInput = new ManualDriveInput();

    private Rotation2d closestBumpTraversalRotation = new Rotation2d(Units.Degrees.of(45));

    /**
     * Constructs a new ManualDriveCommand.
     *
     * @param swerve The swerve subsystem.
     * @param forwardInput Supplier for forward/backward translation input (-1 to 1).
     * @param leftInput Supplier for left/right translation input (-1 to 1).
     * @param rotationInput Supplier for rotational input (-1 to 1).
     */
    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        addRequirements(swerve);
    }

    /**
     * Resets the field-centric heading of the robot to the current facing direction.
     * Useful if the gyro drifts or the driver wants to re-orient "forward".
     */
    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    /**
     * Sets a specific heading to lock onto.
     * Overrides manual rotation and forces the robot to face this direction while translating.
     *
     * @param heading The target heading in Operator Perspective.
     */
    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    /**
     * Captures the current robot heading and sets it as the locked heading.
     * Converts from Blue Alliance perspective (Pose) to Operator Perspective.
     */
    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        setLockedHeading(headingInOperatorPerspective);
    }

    /**
     * Logic to automatically lock the heading if the driver stops rotating for a set duration.
     * This provides a "heading hold" feature.
     *
     * @param input The current drive inputs.
     */
    private void lockHeadingIfRotationStopped(ManualDriveInput input) {
        if (input.hasRotation()) {
            // If rotating, reset the timer and clear locked heading
            headingLockStopwatch.reset();
            lockedHeading = Optional.empty();
        } else {
            // If not rotating, start timer
            headingLockStopwatch.startIfNotRunning();
            // If timer exceeds delay, lock the current heading
            if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) {
                setLockedHeadingToCurrent();
            }
        }
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        headingLockStopwatch.reset();
        previousInput = new ManualDriveInput();
    }

    @Override
    public void execute() {

        // Handle inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        if (input.hasRotation()) {
            // If driver is rotating, prioritize manual rotation
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (input.hasTranslation()) {
            // If translating, check if we should be in locked heading mode
            currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            currentState = State.IDLING;
        }
        previousInput = input;

        // Based on the required control request, this switch case handler changes the state of the drivetrain.
        switch (currentState) {
            case IDLING:
                swerve.setControl(idleRequest);
                break;
            case DRIVING_WITH_MANUAL_ROTATION:
                lockHeadingIfRotationStopped(input);
                swerve.setControl(
                    fieldCentricRequest
                        .withVelocityX(SystemConstants.Drive.kMaxSpeed.times(input.forward))
                        .withVelocityY(SystemConstants.Drive.kMaxSpeed.times(input.left))
                        .withRotationalRate(SystemConstants.Drive.kMaxRotationalRate.times(input.rotation))
                );
                break;
            case DRIVING_WITH_LOCKED_HEADING:
                swerve.setControl(
                    fieldCentricFacingAngleRequest
                        .withVelocityX(SystemConstants.Drive.kMaxSpeed.times(input.forward))
                        .withVelocityY(SystemConstants.Drive.kMaxSpeed.times(input.left))
                        .withTargetDirection(lockedHeading.get())
                );
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}