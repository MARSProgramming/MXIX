package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Floor intake mechanism.
 * Controls the rollers used to intake game pieces from the floor.
 */
public class Floor extends SubsystemBase {

    private TalonFX mFloor;

    // Tunable value for testing percent output via NetworkTables
    private final DoubleSubscriber floorPercentOutTunable = DogLog.tunable("Floor/TunableFloorOutput", 0.8);

    private final StatusSignal<AngularVelocity> mVelocity;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Temperature> mTemp;

    double sTunablePercentOut = floorPercentOutTunable.get();

    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Floor subsystem.
     * Initializes the motor and applies configuration.
     */
    public Floor() {
        mFloor = new TalonFX(Ports.Floor.kFloorRollers);
        mFloor.getConfigurator().apply(SystemConstants.Floor.floorConfig);

        mVelocity = mFloor.getVelocity();
        mVoltage = mFloor.getMotorVoltage();
        mTemp = mFloor.getDeviceTemp();
    }

    /**
     * Sets the floor rollers to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the floor rollers at the specified percent output.
     */


    public Command setPercentOutCommand(double percentOut) {
        return runEnd(() -> {
            mFloor.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFloor.set(0);
        });
    }

    public void setPercentOut(double percentOut) {
        mFloor.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
    }

    /**
     * Sets the floor rollers to the percent output specified by the tunable NetworkTable value.
     * Useful for tuning intake speed without redeploying code.
     *
     * @return A Command that runs the floor rollers at the tunable percent output.
     */
    public Command setPercentOutTunable() {
        return runEnd(() -> {
            mFloor.setControl(floorVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            mFloor.set(0);
        });
    }

    @Override
    public void periodic() {
    sTunablePercentOut = floorPercentOutTunable.get(); // was being reset to 0 — bug fix

    BaseStatusSignal.refreshAll(mVelocity, mVoltage, mTemp);

    DogLog.log("Floor/VelocityRPM", Units.RotationsPerSecond.of(mVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Floor/AppliedVoltage", mVoltage.getValueAsDouble());
    DogLog.log("Floor/Temperature",    mTemp.getValueAsDouble());
    }
}
