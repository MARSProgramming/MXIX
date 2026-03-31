package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Fast Climber mechanism.
 * Uses a TalonFX to control a hooked climber for the endgame.
 */
/**
 * Subsystem: FastClimber
 * Responsible for controlling the FastClimber mechanism.
 */
public class FastClimber extends SubsystemBase {
    TalonFX mFastClimber;

    PositionVoltage fcPositionOut = new PositionVoltage(0);

    private final DoubleSubscriber climbPercentDoubleSubscriber = DogLog.tunable("FastClimber/TunableClimbOutput", 0.4);

    private final StatusSignal<Angle> mPosition;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Temperature> mTemp;

    double cTunableOutput = climbPercentDoubleSubscriber.get();

    public FastClimber() {
        mFastClimber = new TalonFX(Ports.FastClimber.kHookClimber, new CANBus("CAN2"));
        mFastClimber.getConfigurator().apply(SystemConstants.FastClimber.fastClimberConfig);

        mFastClimber.optimizeBusUtilization();

        mFastClimber.getPosition().setUpdateFrequency(50);
        mFastClimber.getMotorVoltage().setUpdateFrequency(10);
        mFastClimber.getDeviceTemp().setUpdateFrequency(4);

        mPosition = mFastClimber.getPosition();
        mVoltage = mFastClimber.getMotorVoltage();
        mTemp = mFastClimber.getDeviceTemp();

        mFastClimber.setPosition(0);
    }

    /**
     * Commands the climber to move to a specified position using voltage-based position control.
     *
     * @param position Target position in rotations.
     * @return Command to execute position tracking.
     */
    public Command setPosition(double position) {
        return runEnd(() -> {
            mFastClimber.setControl(fcPositionOut.withPosition(position));
        }, () -> {
            mFastClimber.set(0);
        });
    }

    /**
     * Commands the climber's motor using an open-loop voltage duty cycle.
     *
     * @param output The fractional duty cycle output (-1.0 to 1.0).
     * @return Command to run the motor at the given duty cycle.
     */
    public Command setPercentOut(double output) {
        return runEnd(() -> {
            mFastClimber.set(output);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    /**
     * Drives the fast climber based on a tunable NetworkTables offset.
     * Useful for safely testing the climbing hooks at varying speeds.
     *
     * @return Command to drive climber out with tunable output.
     */
    public Command setPercentOutTunable() {
        return runEnd(() -> {
            mFastClimber.set(cTunableOutput);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    /**
     * Drives the fast climber in reverse based on the tunable NetworkTables offset.
     *
     * @return Command to pull climber in with tunable output.
     */
    public Command setPercentOutTunableReverse() {
        return runEnd(() -> {
            mFastClimber.set(-cTunableOutput);
        }, () -> {
            mFastClimber.set(0);
        });
    }

    @Override
    public void periodic() {
        cTunableOutput = climbPercentDoubleSubscriber.get();

        BaseStatusSignal.refreshAll(mPosition, mVoltage, mTemp);

        boolean connected = mFastClimber.isConnected(2.0);

        DogLog.log("FastClimber/Position",       mPosition.getValueAsDouble());
        DogLog.log("FastClimber/AppliedVoltage", mVoltage.getValueAsDouble());
        DogLog.log("FastClimber/Temperature",    mTemp.getValueAsDouble());
        DogLog.log("FastClimber/Connected",      connected);

        if (!connected) {
            DogLog.logFault("CAN2: FastClimber Disconnected",
                DriverStation.isFMSAttached() ? null : AlertType.kError);
        } else {
            DogLog.clearFault("CAN2: FastClimber Disconnected");
        }
    }
}