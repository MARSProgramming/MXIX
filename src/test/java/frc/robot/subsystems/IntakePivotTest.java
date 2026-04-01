package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivotTest {

    private IntakePivot pivot;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // Initialize WPILib simulator backend
        pivot = new IntakePivot();
    }

    @Test
    public void testIntakePivotInitialization() {
        assertNotNull(pivot, "IntakePivot subsystem should initialize properly in sim");
    }

    @Test
    public void testDeployCommands() {
        Command cmd = pivot.deployCommand();
        assertNotNull(cmd, "Deploy command should not be null");

        Command retractCmd = pivot.retractCommand();
        assertNotNull(retractCmd, "Retract command should not be null");

        Command slam = pivot.slamtake();
        assertNotNull(slam, "Slamtake command should not be null");
    }
}
