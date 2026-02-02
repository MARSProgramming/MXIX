package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;
/**
 * Subsystem representing the Magazine, which includes the floor rollers and the feeder wheel.
 * This subsystem controls the motors responsible for taking game pieces from the intake area
 * and feeding them into the shooter.
 */
public class Magazine extends SubsystemBase {

    // Feeder motor
    private TalonFX mFeeder;

    // Floor motor
    private TalonFX mFloor;

    // Tunable values for feeder
    private final IntegerSubscriber feederRpmTunable = DogLog.tunable("Magazine/TunableFeederVelocity", 2000);
    double sFeederTunableRpm = feederRpmTunable.get();

    // Tunable values for floor
    private final DoubleSubscriber floorPercentOutTunable = DogLog.tunable("Magazine/TunableFloorOutput", 0.1);
    double sFloorTunablePercentOut = floorPercentOutTunable.get();

    // Control requests for the TalonFX
    VelocityVoltage feederVelocityOut = new VelocityVoltage(0).withSlot(0);
    VoltageOut feederVoltageOut = new VoltageOut(0);
    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Magazine subsystem. Initializes the feeder and floor motors and applies configurations.
     */
    public Magazine() {
        // Feeder motor initialization and configuration
        mFeeder = new TalonFX(Ports.Feeder.kFeederMotor);
        mFeeder.getConfigurator().apply(SystemConstants.Feeder.feederConfig);

        // Floor motor initialization and configuration
        mFloor = new TalonFX(Ports.Floor.kFloorRollers);
        mFloor.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }


    /**
     * Sets the feeder to a specific velocity in RPM.
     * The command runs until interrupted or finished, stopping the motor on end.
     *
     * @param velocityRPM The target velocity in Rotations Per Minute.
     * @return A Command that runs the feeder at the specified velocity.
     */
    public Command setFeederVelocityRPM(double velocityRPM) {
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
    public Command setFeederVelocityRPMTunable() {
        return runEnd(() -> {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(sFeederTunableRpm)));
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
    public Command setFeederVoltagePercentageOut(double percentOut) {
        return runEnd(() -> {
            // Convert percent output to voltage (assuming 12V nominal)
            mFeeder.setControl(feederVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    /**
     * Sets the floor rollers to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the floor rollers at the specified percent output.
     */
    public Command setFloorVoltagePercentageOut(double percentOut) {
        return runEnd(() -> {
            mFloor.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFloor.set(0);
        });
    }

    /**
     * Sets the floor rollers to the percent output specified by the tunable NetworkTable value.
     * Useful for tuning intake speed without redeploying code.
     *
     * @return A Command that runs the floor rollers at the tunable percent output.
     */
    public Command setFloorVoltagePercentOutTunable() {
        return runEnd(() -> {
            mFloor.setControl(floorVoltageOut.withOutput(Units.Volts.of(sFloorTunablePercentOut * 12.0)));
        }, () -> {
            mFloor.set(0);
        });
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

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        sFeederTunableRpm = feederRpmTunable.get();
        sFloorTunablePercentOut = floorPercentOutTunable.get();

        // Log relevant data to DogLog/NetworkTables
        DogLog.log("Magazine/Feeder/VelocityRPM", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Magazine/Feeder/AppliedVoltage", mFeeder.getMotorVoltage().getValueAsDouble());
        DogLog.log("Magazine/Feeder/Temperature", mFeeder.getDeviceTemp().getValueAsDouble());
        DogLog.log("Magazine/Feeder/TunableFeederVelocity", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Magazine/Floor/VelocityRPM", Units.RotationsPerSecond.of(mFloor.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Magazine/Floor/AppliedVoltage", mFloor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Magazine/Floor/Temperature", mFloor.getDeviceTemp().getValueAsDouble());
    }

}
