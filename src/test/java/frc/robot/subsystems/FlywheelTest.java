package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelTest {

    private Flywheel flywheel;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // Initialize WPILib simulator backend
        flywheel = new Flywheel();
    }

    @Test
    public void testFlywheelInitialization() {
        assertNotNull(flywheel, "Flywheel subsystem should initialize properly in sim");
    }

    @Test
    public void testShooterCommandReturnsNonNull() {
        Command cmd = flywheel.setVelocity(() -> 3000.0);
        assertNotNull(cmd, "Shoot command should not be null");
    }

    @Test
    public void testSetVelocityDoesNotThrow() {
        // Just executing to ensure it doesn't crash
        flywheel.setRPM(2000.0);
    }
}
