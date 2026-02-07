package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Cowl mechanism.
 * This subsystem controls the position of the cowl using a TalonFX motor.
 */
public class IntakePivot extends SubsystemBase {
    TalonFX mIntakePivot;

    // Control request for position control using voltage
    PositionVoltage cowlPositionOut = new PositionVoltage(0);

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Intake/TunablePosition", 0.1);
    double cTunablePosition = cowlPositionTunable.get();

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public IntakePivot() {
        mIntakePivot = new TalonFX(Ports.Intake.kIntakePivot);
        mIntakePivot.getConfigurator().apply(SystemConstants.Intake.pivotConfig);
    }

    /**
     * Sets the cowl to a specific position.
     * The command runs until interrupted or finished, stopping the motor on end.
     *
     * @param position The target position in rotations.
     * @return A Command that moves the cowl to the specified position.
     */
    public Command setPosition(double position) {
        return runEnd(() -> {
            mIntakePivot.setControl(cowlPositionOut.withPosition(position));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    /**
     * Sets the cowl to the position specified by the tunable NetworkTable value.
     * Useful for tuning the position setpoint without redeploying code.
     *
     * @return A Command that moves the cowl to the tunable position.
     */
    public Command setPositionTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(cowlPositionOut.withPosition(cTunablePosition));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    public Command home() {
        return run(() -> {
            mIntakePivot.set(SystemConstants.Cowl.kCowlHomingOutput);
        }).until(
            () -> mIntakePivot.getSupplyCurrent().getValueAsDouble() < SystemConstants.Intake.kIntakePivotStallCurrent
        );
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunablePosition = cowlPositionTunable.get();

        // Log current position
        DogLog.log("Cowl/Position", mIntakePivot.getPosition().getValueAsDouble());
        DogLog.log("Cowl/AppliedVoltage", mIntakePivot.getMotorVoltage().getValueAsDouble());
        DogLog.log("Cowl/Temperature", mIntakePivot.getDeviceTemp().getValueAsDouble());
        DogLog.log("Cowl/TunablePosition", cTunablePosition);
    }
}
