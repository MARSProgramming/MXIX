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
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Floor intake mechanism.
 * Controls the rollers used to intake game pieces from the floor.
 */
public class IntakeRollers extends SubsystemBase {

    private TalonFX mIntakeRollers;

    // Tunable value for testing percent output via NetworkTables
    private final DoubleSubscriber intakePercentOutTunable = DogLog.tunable("Intake/TunableIntakeRollerOutput", 0.8);
    double sTunablePercentOut = intakePercentOutTunable.get();

    private final StatusSignal<AngularVelocity> mVelocity = mIntakeRollers.getVelocity();
    private final StatusSignal<Voltage> mVoltage  = mIntakeRollers.getMotorVoltage();
    private final StatusSignal<Temperature> mTemp     = mIntakeRollers.getDeviceTemp();

    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Floor subsystem.
     * Initializes the motor and applies configuration.
     */
    public IntakeRollers() {
        mIntakeRollers = new TalonFX(Ports.Intake.kIntakeRollers);
        mIntakeRollers.getConfigurator().apply(SystemConstants.Intake.rollerConfig);
    }

    /**
     * Sets the floor rollers to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the floor rollers at the specified percent output.
     */
    public Command setPercentOutCommand(double percentOut) {
        return runEnd(() -> {
            mIntakeRollers.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mIntakeRollers.set(0);
        });
    }

    public Command intakeCommand() {
        return runEnd(() -> {
            mIntakeRollers.setControl(floorVoltageOut.withOutput(Units.Volts.of(Settings.IntakeSystemSettings.INTAKING_STANDARD_DUTYCYCLE * 12.0)));
        }, () -> {
            mIntakeRollers.set(0);
        }); 
    }

    public void setPercentOut(double percentOut) {
        mIntakeRollers.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
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
        sTunablePercentOut = intakePercentOutTunable.get(); // was being reset to 0 — bug fix

        BaseStatusSignal.refreshAll(mVelocity, mVoltage, mTemp);

        DogLog.log("Intake/VelocityRPM", Units.RotationsPerSecond.of(mVelocity.getValueAsDouble()).in(Units.RPM));
        DogLog.log("Intake/AppliedVoltage", mVoltage.getValueAsDouble());
        DogLog.log("Intake/Temperature",    mTemp.getValueAsDouble());
    }
}
