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
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.GeometryUtil;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class PrepareSupershot extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final Flywheel flywheel;
    private final Cowl cowl;
    private final ShotSetup setup;
    

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
    public PrepareSupershot(
        ShotSetup setup,
        Swerve swerve,
        Flywheel flywheel,
        Cowl cowl,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.setup = setup;
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.cowl = cowl;

        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, flywheel, cowl);
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


    @Override
    public void execute() {
        // Get smoothed joystick inputs
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        ShotSetup.SOTMInfo currentInfo = setup.getSOTMInfo(swerve);

        // Apply control request to swerve
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Drive.kMaxSpeed.times(input.forward))
                .withVelocityY(Drive.kMaxSpeed.times(input.left))
                .withTargetDirection(currentInfo.virtualTargetAngle) // Continuously update target direction
        );

        flywheel.setRPM(currentInfo.shotInfo.shot.shooterRPM);
        cowl.setPosition(currentInfo.shotInfo.cowlPosition);        
    }

    public boolean readyToShoot() {
        return (isAimed() && flywheel.isVelocityWithinTolerance());
    }

    
    @Override
    public void end(boolean interrupted) {
        cowl.home();
        flywheel.setPercentOut(0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }
}