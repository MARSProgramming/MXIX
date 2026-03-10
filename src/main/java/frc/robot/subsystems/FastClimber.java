package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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
    private final DoubleSubscriber climbPercentDoubleSubscriber = DogLog.tunable("FastClimber/TunableClimbOutput", 0.4);

    private final StatusSignal<Angle> mPosition = mFastClimber.getPosition();
    private final StatusSignal<Voltage> mVoltage   = mFastClimber.getMotorVoltage();
    private final StatusSignal<Temperature> mTemp      = mFastClimber.getDeviceTemp();

    double cTunableOutput = climbPercentDoubleSubscriber.get();

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public FastClimber() {
        mFastClimber = new TalonFX(Ports.FastClimber.kHookClimber);
        mFastClimber.getConfigurator().apply(SystemConstants.FastClimber.fastClimberConfig);
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

    public Command setPercentOut(double output) {
        return runEnd(() -> {
            mFastClimber.set(output);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    public Command setPercentOutTunable() {
        return runEnd(() -> {
            mFastClimber.set(cTunableOutput);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    public Command setPercentOutTunableReverse() {
        return runEnd(() -> {
            mFastClimber.set(-cTunableOutput);
        }, () -> {
            mFastClimber.set(0);
        });
    }


    @Override
    public void periodic() {
    // Read tunables from NetworkTables
    cTunableOutput   = climbPercentDoubleSubscriber.get(); // was never being updated — bug fix

    // Batch refresh all CAN signals in one JNI call
    BaseStatusSignal.refreshAll(mPosition, mVoltage, mTemp);

    DogLog.log("FastClimber/Position",       mPosition.getValueAsDouble());
    DogLog.log("FastClimber/AppliedVoltage", mVoltage.getValueAsDouble());
    DogLog.log("FastClimber/Temperature",    mTemp.getValueAsDouble());

    }
}
