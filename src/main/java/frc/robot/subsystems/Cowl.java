package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Cowl mechanism.
 * This subsystem controls the position of the cowl using a TalonFX motor.
 */
public class Cowl extends SubsystemBase {
    TalonFX mCowl;

    double COWL_POSITION_TOLERANCE = 0.05; // Tolerance in rotations for considering the cowl "at position"

    // Control request for position control using voltage
    PositionVoltage cowlPositionOut = new PositionVoltage(0).withSlot(0);

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Cowl/TunableCowlPosition", 0.5);
    private final DoubleSubscriber cowlPercentOutTunable = DogLog.tunable("Cowl/TunableCowlPercentout", 0.2);
    double cTunablePosition = 0;
    double cTunablePercentout = 0;
    // Rate limiter for logging to avoid overwhelming the robot thread
    private int logCounter = 0;

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public Cowl() {
        mCowl = new TalonFX(Ports.Cowl.kCowlMotor);
        mCowl.getConfigurator().apply(SystemConstants.Cowl.cowlConfig);

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

        
    public Command setPositionCommand(DoubleSupplier position) {
        return runEnd(() -> {
            mCowl.setControl(cowlPositionOut.withPosition(position.getAsDouble()));
        }, () -> {
            mCowl.set(0);
        });
    }

    public Command setPositionContinuously(double position) {
        return run(() -> {
            mCowl.setControl(cowlPositionOut.withPosition(position));
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

    // Re-zero the cowl motor

    public Command home() {
        return run(() -> {
            mCowl.set(SystemConstants.Cowl.kCowlHomingOutput);
        }).until(
            () -> mCowl.getSupplyCurrent().getValueAsDouble() > SystemConstants.Cowl.kCowlStallCurrent
        ).andThen(runOnce(() -> {
            mCowl.setPosition(0);
            mCowl.set(0);
        }));
    }

    public Command zero() {
        return runOnce(() -> mCowl.setPosition(0));
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunablePosition = 0;
        cTunablePercentout = 0;

        // Rate-limited logging: log at ~10Hz instead of 50Hz
        if (++logCounter >= 5) {
            logCounter = 0;
            DogLog.log("Cowl/Position", mCowl.getPosition().getValueAsDouble());
            DogLog.log("Cowl/AppliedVoltage", mCowl.getMotorVoltage().getValueAsDouble());
            DogLog.log("Cowl/SupplyCurrent", mCowl.getSupplyCurrent().getValueAsDouble());
            DogLog.log("Cowl/Temperature", mCowl.getDeviceTemp().getValueAsDouble());
        }
    }
}
