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
public class FastClimber extends SubsystemBase {
    TalonFX mFastClimber;

    // Control request for position control using voltage
    PositionVoltage fcPositionOut = new PositionVoltage(0);

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber clibmPercentOutTunable = DogLog.tunable("FastClimber/TunableClimbOutput", 0.1);
    private final DoubleSubscriber climbPositionTunable = DogLog.tunable("FastClimber/TunableClimbPosition", 0.1);

    double cTunableOutput = clibmPercentOutTunable.get();
    double cTunablePosition = climbPositionTunable.get();

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public FastClimber() {
        mFastClimber = new TalonFX(Ports.FastClimber.kHookClimber);
        mFastClimber.getConfigurator().apply(SystemConstants.Cowl.cowlConfig);
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
            mFastClimber.setControl(fcPositionOut.withPosition(position));
        }, () -> {
            mFastClimber.set(0);
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
            mFastClimber.setControl(fcPositionOut.withPosition(cTunablePosition));
        }, () -> {
            mFastClimber.set(0);
        });
    }

    public Command setPercentOut(double output) {
        return runEnd(() -> {
            mFastClimber.set(output);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    public Command setPercentOutTunable() {
        return runEnd(() -> {
            mFastClimber.set(clibmPercentOutTunable.get());
        }, () -> {
            mFastClimber.set(0);
        });
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunablePosition = climbPositionTunable.get();
        cTunableOutput = clibmPercentOutTunable.get();

        // Log current position
        DogLog.log("FastClimber/Position", mFastClimber.getPosition().getValueAsDouble());
        DogLog.log("FastClimber/AppliedVoltage", mFastClimber.getMotorVoltage().getValueAsDouble());
        DogLog.log("FastClimber/Temperature", mFastClimber.getDeviceTemp().getValueAsDouble());
        DogLog.log("FastClimber/TunablePercentOut", cTunableOutput);
        DogLog.log("FastClimber/TunablePosition", cTunablePosition);

    }
}
