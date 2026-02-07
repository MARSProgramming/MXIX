package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Floor intake mechanism.
 * Controls the rollers used to intake game pieces from the floor.
 */
public class IntakeRollers extends SubsystemBase {

    private TalonFX mIntakeRollers;

    // Tunable value for testing percent output via NetworkTables
    private final DoubleSubscriber intakePercentOutTunable = DogLog.tunable("Intake/TunableIntakeRollerOutput", 0.1);
    double sTunablePercentOut = intakePercentOutTunable.get();

    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Floor subsystem.
     * Initializes the motor and applies configuration.
     */
    public IntakeRollers() {
        mIntakeRollers = new TalonFX(Ports.Floor.kFloorRollers);
        mIntakeRollers.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }

    /**
     * Sets the floor rollers to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the floor rollers at the specified percent output.
     */
    public Command set(double percentOut) {
        return runEnd(() -> {
            mIntakeRollers.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mIntakeRollers.set(0);
        });
    }

    /**
     * Sets the floor rollers to the percent output specified by the tunable NetworkTable value.
     * Useful for tuning intake speed without redeploying code.
     *
     * @return A Command that runs the floor rollers at the tunable percent output.
     */
    public Command setTunable() {
        return runEnd(() -> {
            mIntakeRollers.setControl(floorVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            mIntakeRollers.set(0);
        });
    }

    @Override
    public void periodic() {
        // Update local tunable variable and log data
        sTunablePercentOut = intakePercentOutTunable.get();
        DogLog.log("Intake/VelocityRPM", Units.RotationsPerSecond.of(mIntakeRollers.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Intake/AppliedVoltage", mIntakeRollers.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Temperature", mIntakeRollers.getDeviceTemp().getValueAsDouble());
        DogLog.log("Intake/TunableFeederVelocity", Units.RotationsPerSecond.of(mIntakeRollers.getVelocity().getValueAsDouble()).in(Units.RPM));
    }
}
