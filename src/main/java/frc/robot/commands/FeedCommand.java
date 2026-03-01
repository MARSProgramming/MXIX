package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.IntakeRollers;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class FeedCommand extends Command {

    private final IntakeRollers intakeRollers;
    private final Feeder feeder;
    private final Floor floor;
    



    /**
     * Constructs a new AimAndDriveCommand.
     *
     * @param swerve The swerve subsystem.
     * @param forwardInput Supplier for forward/backward translation input (-1 to 1).
     * @param leftInput Supplier for left/right translation input (-1 to 1).
     */
    public FeedCommand(
        IntakeRollers intakeRollers,
        Feeder feeder,
        Floor floor
    ) {
        this.intakeRollers = intakeRollers;
        this.feeder = feeder;
        this.floor = floor;

        addRequirements(intakeRollers, feeder, floor);
    }

    @Override
    public void execute() {
        intakeRollers.set(0.5);
        feeder.setPercentOut(0.7);
        floor.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeRollers.set(0);
        floor.set(0);
        feeder.setPercentOut(0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }
}