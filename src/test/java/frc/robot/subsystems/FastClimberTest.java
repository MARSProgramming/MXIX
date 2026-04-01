package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;

public class FastClimberTest {

    private FastClimber climber;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // Initialize WPILib simulator backend
        climber = new FastClimber();
    }

    @Test
    public void testClimberInitialization() {
        assertNotNull(climber, "Climber subsystem should initialize properly in sim");
    }

    @Test
    public void testClimberCommands() {
        Command climb = climber.setPosition(1.0);
        assertNotNull(climb, "Climb command should not be null");

        Command unwind = climber.setPercentOut(-0.5);
        assertNotNull(unwind, "Unwind command should not be null");
    }

}
