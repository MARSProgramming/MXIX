package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber pivotPercentOutTunable = DogLog.tunable("IntakePivot/TunablePercentOut", 0.5);
    double cTunablePivotOut = pivotPercentOutTunable.get();

    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public IntakePivot() {
        mIntakePivot = new TalonFX(Ports.Intake.kIntakePivot);
        mIntakePivot.getConfigurator().apply(SystemConstants.Intake.pivotConfig);
    }


    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(percentOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    public Command forwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(cTunablePivotOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

        public Command backwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(-cTunablePivotOut * 12.0));
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
        cTunablePivotOut = pivotPercentOutTunable.get();

        // Log current position
        DogLog.log("Intake/Pivot/Position", mIntakePivot.getPosition().getValueAsDouble());
        DogLog.log("Intake/Pivot/AppliedVoltage", mIntakePivot.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Pivot/Temperature", mIntakePivot.getDeviceTemp().getValueAsDouble());
    }
}