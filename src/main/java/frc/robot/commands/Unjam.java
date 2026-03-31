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
/**
 * Command: Unjam
 * Executes the Unjam action.
 */
public class Unjam extends Command {

    private final Feeder feeder;
    private final IntakeRollers intakeRollers;
    private final Floor floor;


    /**
     * Constructs a new AimAndDriveCommand.
     *
     * @param swerve The swerve subsystem.
     * @param forwardInput Supplier for forward/backward translation input (-1 to 1).
     * @param leftInput Supplier for left/right translation input (-1 to 1).
     */
    public Unjam(
        Feeder feeder,
        Floor floor,
        IntakeRollers intakeRollers
    ) {
        this.feeder = feeder;
        this.floor = floor;
        this.intakeRollers = intakeRollers;

        addRequirements(feeder, floor, intakeRollers);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Get smoothed joystick inputs
            feeder.setPercentOut(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.UNJAM_DUTYCYCLE);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        intakeRollers.setPercentOut(0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }
}