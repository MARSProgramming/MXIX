package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.GeometryUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Cowl mechanism.
 * This subsystem controls the position of the cowl using a TalonFX motor.
 */
/**
 * Subsystem: Cowl
 * Responsible for controlling the Cowl mechanism.
 */
public class Cowl extends SubsystemBase {
    TalonFX mCowl;

    double COWL_POSITION_TOLERANCE = 0.1; // Tolerance in rotations for considering the cowl "at position"
    // can trigger early, because it takes about 200ms to get to position.

    // Control request for position control using voltage
    PositionVoltage cowlPositionOut = new PositionVoltage(0).withSlot(0);

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Cowl/TunableCowlPosition", 0.6);
    private final DoubleSubscriber cowlPercentOutTunable = DogLog.tunable("Cowl/TunableCowlPercentout", 0.2);

    double cTunablePosition = cowlPositionTunable.get();
    double cTunablePercentout = cowlPercentOutTunable.get();

    private final StatusSignal<Angle> mPosition;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Current> mSupplyCurrent;
    private final StatusSignal<Temperature> mTemp;

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public Cowl() {
        mCowl = new TalonFX(Ports.Cowl.kCowlMotor, "CAN2");
        mCowl.getConfigurator().apply(SystemConstants.Cowl.cowlConfig);

        mCowl.optimizeBusUtilization();

        mCowl.getPosition().setUpdateFrequency(50);
        mCowl.getMotorVoltage().setUpdateFrequency(10);
        mCowl.getSupplyCurrent().setUpdateFrequency(10);
        mCowl.getDeviceTemp().setUpdateFrequency(4);

        mPosition = mCowl.getPosition();
        mVoltage = mCowl.getMotorVoltage();
        mSupplyCurrent = mCowl.getSupplyCurrent();
        mTemp = mCowl.getDeviceTemp();

        mCowl.setPosition(0);
    }

    /**
     * Sets the cowl to a specific position.
     * The command runs until interrupted or finished, stopping the motor on end.
     *
     * @param position The target position in rotations.
     * @return A Command that moves the cowl to the specified position.
     */
    public void setPosition(double position) {
        mCowl.setControl(cowlPositionOut.withPosition(position));
    }

        
    /**
     * Provides a dynamic command to set the cowl position based on a supplier.
     *
     * @param position DoubleSupplier providing real-time target position.
     * @return A Command tracking position continually while executing.
     */
    public Command setPositionCommand(DoubleSupplier position) {
        return runEnd(() -> {
            mCowl.setControl(cowlPositionOut.withPosition(position.getAsDouble()));
        }, () -> {
            mCowl.set(0);
        });
    }

    /**
     * Returns a command that continually holds a specific cowl position while running.
     * Does not zero motor output on command end.
     *
     * @param position Target rotations.
     * @return Command running position feedforward loop.
     */
    public Command setPositionContinuously(double position) {
        return run(() -> {
            mCowl.setControl(cowlPositionOut.withPosition(position));
        });
    }

    /**
     * Verifies if the cowl is tracking close to its setpoint.
     *
     * @param setpoint The target position.
     * @return boolean True if within tolerance.
     */
    public boolean isAtTolerance(double setpoint) {
        double currPos = mPosition.getValueAsDouble();
         return MathUtil.isNear(setpoint, currPos, COWL_POSITION_TOLERANCE);
    }
    


    /**
     * Sets the cowl to the position specified by the tunable NetworkTable value.
     * Useful for tuning the position setpoint without redeploying code.
     *
     * @return A Command that moves the cowl to the tunable position.
     */
    public Command setPositionTunable() {
        return runEnd(() -> {
            double desiredCowlPos = cowlPositionTunable.get();
            mCowl.setControl(cowlPositionOut.withPosition(desiredCowlPos));
        }, () -> {
            mCowl.set(0);
        });
    }

    public Command forwardTunable() {
        return runEnd(() -> {
            mCowl.set(cTunablePercentout);
        }, () -> {
            mCowl.set(0);
        });
    }

    public Command backwardTunable() {
        return runEnd(() -> {
            mCowl.set(-cTunablePercentout);
        }, () -> {
            mCowl.set(0);
        });
    }

    /**
     * Executes a homing sequence for the Cowl motor by driving it gently into a hardstop,
     * stalling, and recording 0 before commanding a stop.
     *
     * @return Command representing homing routine.
     */
    public Command home() {
        return run(() -> {
            mCowl.set(SystemConstants.Cowl.kCowlHomingOutput);
        }).withTimeout(SystemConstants.Cowl.kCowlStallTimeout)
        .andThen(runOnce(() -> {
            mCowl.setPosition(0);
            mCowl.set(0);
        }));
    }
    /**
     * Strictly re-centers the cowl motor encoder internally without moving it.
     *
     * @return Command to zero rotor encoder.
     */
    public Command zero() {
        return runOnce(() -> mCowl.setPosition(0));
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunablePosition = cowlPositionTunable.get();
        cTunablePercentout = cowlPercentOutTunable.get();
        // Batch refresh all CAN signals in one JNI call
        BaseStatusSignal.refreshAll(mPosition, mVoltage, mSupplyCurrent, mTemp);
        boolean connected = mCowl.isConnected(2.0);

        DogLog.log("Cowl/Position",       mPosition.getValueAsDouble());
        DogLog.log("Cowl/AppliedVoltage", mVoltage.getValueAsDouble());
        DogLog.log("Cowl/SupplyCurrent",  mSupplyCurrent.getValueAsDouble());
        DogLog.log("Cowl/Temperature",    mTemp.getValueAsDouble());
        DogLog.log("Cowl/Connected",      connected);

        if (!connected) {
            DogLog.logFault("CAN2: Cowl Disconnected",
            DriverStation.isFMSAttached() ? null : AlertType.kError);
        } else {
            DogLog.clearFault("CAN2: Cowl Disconnected");
        }
    }
}
