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
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class FastClimber extends SubsystemBase {
    TalonFX mFastClimber;

    PositionVoltage fcPositionOut = new PositionVoltage(0);

    private final DoubleSubscriber climbPercentDoubleSubscriber = DogLog.tunable("FastClimber/TunableClimbOutput", 0.4);

    private final StatusSignal<Angle> mPosition;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Temperature> mTemp;

    double cTunableOutput = climbPercentDoubleSubscriber.get();

    public FastClimber() {
        mFastClimber = new TalonFX(Ports.FastClimber.kHookClimber, "CAN2");
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