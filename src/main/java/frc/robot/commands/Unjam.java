package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;

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