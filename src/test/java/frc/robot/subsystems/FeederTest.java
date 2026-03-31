package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;

public class FeederTest {

    private Feeder feeder;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // Initialize WPILib simulator backend
        feeder = new Feeder();
    }

    @Test
    public void testFeederInitialization() {
        assertNotNull(feeder, "Feeder subsystem should initialize properly in sim");
    }

    @Test
    public void testFeederCommands() {
        Command fwd = feeder.setVelocity(1000);
        assertNotNull(fwd, "Forward command should not be null");

        Command bck = feeder.setPercentOutCommand(-0.5);
        assertNotNull(bck, "Backward command should not be null");
    }

    @Test
    public void testSetRPM() {
        feeder.setRPM(1000);
    }
}
