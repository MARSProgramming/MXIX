package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Flywheel;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
/**
 * Command: ShootOnly
 * Executes the ShootOnly action.
 */
public class ShootOnly extends Command {
    private final Cowl cowl;
    private final Flywheel flywheel;
    private final double cowlInput;
    private final double velocityInput;

    public ShootOnly(
        Cowl cowl,
        Flywheel flywheel,
        double velocityInput,
        double cowlInput
    ) {
        this.cowl = cowl;
        this.flywheel = flywheel;
        this.cowlInput = cowlInput;
        this.velocityInput = velocityInput;

        addRequirements(cowl, flywheel);
    }




    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        cowl.setPosition(cowlInput);
        flywheel.setRPM(velocityInput);

    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(cowl.home());
        flywheel.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }

}