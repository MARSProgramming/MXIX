package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

    private final IntegerSubscriber feederRpmTunable = DogLog.tunable("Feeder/TunableFeederVelocity", 2000);

    double sTunableRpm = feederRpmTunable.get();

    // Control requests for the TalonFX
    VelocityVoltage feederVelocityOut = new VelocityVoltage(0).withSlot(0);
    VoltageOut feederVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Feeder subsystem.
     * Initializes the motor and applies configuration.
     */
    public Feeder() {
        mFeeder = new TalonFX(Ports.Feeder.kFeederMotor);
        // Apply the configuration.
        mFeeder.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }

    /**
     * set feeder RPM
     * @param velocityRPM
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
     * Sets the feeder to the velocity specified by the tunable NetworkTable value.
     * Useful for tuning the RPM without redeploying code.
     *
     * @return A Command that runs the feeder at the tunable velocity.
     */
    public Command setVelocityTunable() {
        return runEnd(() -> {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
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
    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            // Convert percent output to voltage (assuming 12V nominal)
            mFeeder.setControl(feederVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        sTunableRpm = feederRpmTunable.get();
        
        // Log relevant data to DogLog/NetworkTables
        DogLog.log("Feeder/VelocityRPM", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Feeder/AppliedVoltage", mFeeder.getMotorVoltage().getValueAsDouble());
        DogLog.log("Feeder/Temperature", mFeeder.getDeviceTemp().getValueAsDouble());
        DogLog.log("Feeder/TunableFeederVelocity", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
    }

    /**
     * Checks if the feeder motor is within the configured velocity tolerance of the target.
     *
     * @return true if the motor is in velocity control mode and near the target velocity.
     */
    public boolean isVelocityWithinTolerance() {
        // Check if the last applied control request was the velocity request
        final boolean isInVelocityMode = mFeeder.getAppliedControl().equals(feederVelocityOut);
        final AngularVelocity currentVelocity = mFeeder.getVelocity().getValue();
        final AngularVelocity targetVelocity = feederVelocityOut.getVelocityMeasure();
        
        // Return true only if we are trying to control velocity and are close enough
        return isInVelocityMode && currentVelocity.isNear(targetVelocity, SystemConstants.Feeder.kVelocityTolerance);
    }
}
