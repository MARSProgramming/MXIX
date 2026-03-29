package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Feeder mechanism.
 * This subsystem controls the motor responsible for feeding game pieces into the shooter/flywheel.
 */
public class Feeder extends SubsystemBase {

    private TalonFX mFeeder;

    private final DoubleSubscriber feederPercOutTunable = DogLog.tunable("Feeder/TunableFeederPercOut", 1.0);

    private final StatusSignal<AngularVelocity> mVelocity;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Temperature> mTemp;

    double stunablePercOut = feederPercOutTunable.get(); 

    // Control requests for the TalonFX
    VelocityVoltage feederVelocityOut = new VelocityVoltage(0).withSlot(0);
    VoltageOut feederVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Feeder subsystem.
     * Initializes the motor and applies configuration.
     */
    public Feeder() {
        mFeeder = new TalonFX(Ports.Feeder.kFeederMotor, "CAN2");
        // Apply the configuration.
        mFeeder.getConfigurator().apply(SystemConstants.Feeder.feederConfig);

        mFeeder.optimizeBusUtilization();

        mFeeder.getVelocity().setUpdateFrequency(50);
        mFeeder.getMotorVoltage().setUpdateFrequency(10);
        mFeeder.getDeviceTemp().setUpdateFrequency(4);


        mVelocity = mFeeder.getVelocity();
        mVoltage = mFeeder.getMotorVoltage();
        mTemp = mFeeder.getDeviceTemp();
    }

    /**
     * Immediately sets the feeder's target velocity in RPM.
     * This is not a command, but a direct set.
     * 
     * @param velocityRPM The desired velocity in Rotations Per Minute.
     */
    
    public void setRPM(double velocityRPM) {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
    }

    /**
     * Sets the feeder to a specific velocity in RPM.
     * The command runs until interrupted or finished, stopping the motor on end.
     *
     * @param velocityRPM The target velocity in Rotations Per Minute.
     * @return A Command that runs the feeder at the specified velocity.
     */
    public Command setVelocity(double velocityRPM) {
        return runEnd(() -> {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
        }, () -> {
            mFeeder.set(0);
        });
    }

   
    /**
     * Runs the feeder motor at a duty cycle value defined by tunable NetworkTables.
     * Allows live tuning of the feeder.
     *
     * @return Command that drives feeder at tunable percent output.
     */
    public Command setPercentOutTunable() {
        return runEnd(() -> {
            mFeeder.set(stunablePercOut);
        }, () -> {
            mFeeder.set(0);
        });
    }


    /**
     * Sets the feeder motor to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the feeder at the specified percent output.
     */
    public Command setPercentOutCommand(double percentOut) {
        return runEnd(() -> {
            // Convert percent output to voltage (assuming 12V nominal)
            mFeeder.setControl(feederVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    /**
     * Directly sets the feeder motor to a percentage of output voltage.
     *
     * @param percentOut The target duty cycle (-1.0 to 1.0).
     */
    public void setPercentOut(double percentOut) {
        mFeeder.setControl(feederVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
    }

    @Override
    public void periodic() {
    stunablePercOut = feederPercOutTunable.get();

    BaseStatusSignal.refreshAll(mVelocity, mVoltage, mTemp);

    boolean connected = mFeeder.isConnected(2.0);

    DogLog.log("Feeder/VelocityRPM",Units.RotationsPerSecond.of(mVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Feeder/AppliedVoltage", mVoltage.getValueAsDouble());
    DogLog.log("Feeder/Temperature",    mTemp.getValueAsDouble());
    DogLog.log("Feeder/Connected",      connected);

    if (!connected) {
        DogLog.logFault("CAN2: Feeder Disconnected",
            DriverStation.isFMSAttached() ? null : AlertType.kError);
    } else {
        DogLog.clearFault("CAN2: Feeder Disconnected");
    }
}

    /**
     * Checks if the feeder motor is within the configured velocity tolerance of the target.
     *
     * @return true if the motor is in velocity control mode and near the target velocity.
     */
    
    public boolean isVelocityWithinTolerance() {
    final boolean isInVelocityMode = mFeeder.getAppliedControl().equals(feederVelocityOut);
    final AngularVelocity currentVelocity = mVelocity.getValue(); // use cached signal
    final AngularVelocity targetVelocity = feederVelocityOut.getVelocityMeasure();

    return isInVelocityMode && currentVelocity.isNear(targetVelocity, SystemConstants.Feeder.kVelocityTolerance);
    }
}
