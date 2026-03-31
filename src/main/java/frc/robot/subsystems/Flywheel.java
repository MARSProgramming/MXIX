package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Subsystem representing the Flywheel (Shooter) mechanism.
 * Controls the four motors (2 masters, 2 followers) used to launch game pieces.
 */
/**
 * Subsystem: Flywheel
 * Responsible for controlling the Flywheel mechanism.
 */
public class Flywheel extends SubsystemBase {

    private TalonFX rm;
    private TalonFX rf;
    private TalonFX lm;
    private TalonFX lf;


    // Tunable values for testing velocity and percent output via NetworkTables
    private final DoubleSubscriber shooterRpmTunable = DogLog.tunable("Shooter/TunableShooterVelocity", 3400.0);
    private final DoubleSubscriber shooterPercentOutTunable = DogLog.tunable("Shooter/TunableShooterOutput", 0.5);


    private final StatusSignal<AngularVelocity> rmVelocity, rfVelocity, lmVelocity, lfVelocity;
    private final StatusSignal<Current> rmCurrent, lmCurrent;
    private final StatusSignal<Temperature> rmTemp, rfTemp, lmTemp, lfTemp;

    double sTunableRpm = shooterRpmTunable.get();
    double sTunablePercentOut = shooterPercentOutTunable.get();

    // Control requests
    VoltageOut flywheelVoltageOut = new VoltageOut(0);
    VelocityVoltage flywheelVelocityOut = new VelocityVoltage(0).withSlot(0);

    /**
     * Creates a new Flywheel subsystem.
     * Initializes motors, applies configurations, and sets up follower relationships.
     */
    public Flywheel() {
        rm = new TalonFX(Ports.Flywheel.kRightFlywheelMaster, "CAN2");
        rf = new TalonFX(Ports.Flywheel.kRightFlywheelFollower, "CAN2");
        lm = new TalonFX(Ports.Flywheel.kLeftFlywheelMaster, "CAN2");
        lf = new TalonFX(Ports.Flywheel.kLeftFlywheelFollower, "CAN2");

        rm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        rf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);

        

        // Configure followers to oppose the masters (since they are on opposite sides/gearing)
        rf.setControl(new Follower(Ports.Flywheel.kRightFlywheelMaster, MotorAlignmentValue.Opposed));
        lf.setControl(new Follower(Ports.Flywheel.kLeftFlywheelMaster, MotorAlignmentValue.Opposed));

        // Masters - need velocity for closed loop control
    rm.optimizeBusUtilization();
    lm.optimizeBusUtilization();

    rm.getVelocity().setUpdateFrequency(50);
    rm.getSupplyCurrent().setUpdateFrequency(20);
    rm.getDeviceTemp().setUpdateFrequency(4);

    lm.getVelocity().setUpdateFrequency(50);
    lm.getSupplyCurrent().setUpdateFrequency(20);
    lm.getDeviceTemp().setUpdateFrequency(4);

        // Followers - minimal signals, they just follow
    rf.optimizeBusUtilization();
    lf.optimizeBusUtilization();

    rf.getVelocity().setUpdateFrequency(20);
    rf.getDeviceTemp().setUpdateFrequency(4);

    lf.getVelocity().setUpdateFrequency(20);
    lf.getDeviceTemp().setUpdateFrequency(4);

        rmVelocity = rm.getVelocity();
        rfVelocity = rf.getVelocity();
        lmVelocity = lm.getVelocity();
        lfVelocity = lf.getVelocity();
        rmCurrent  = rm.getSupplyCurrent();
        lmCurrent  = lm.getSupplyCurrent();
        rmTemp     = rm.getDeviceTemp();
        rfTemp     = rf.getDeviceTemp();
        lmTemp     = lm.getDeviceTemp();
        lfTemp     = lf.getDeviceTemp();
    }

    /**
     * Obtains the target velocity for the flywheels in RPM.
     * This method is an empty stub in this format/version but preserved for structure.
     *
     * @return A Command that sets the RPM and waits for the flywheel to be within tolerance.
     */
    
    /**
     * Sets the target velocity for the flywheels in RPM.
     * This method sets the control request but does not manage the command lifecycle.
     *
     * @param velocityRPM The target velocity in Rotations Per Minute.
     */
    public void setRPM(double velocityRPM) {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
    }

    /**
     * Gets the current velocity of the right master flywheel.
     * 
     * @return The angular velocity of the flywheel.
     */
    public AngularVelocity getVelocity() {
        return rm.getVelocity().getValue();          
    }

    /**
     * Sets the flywheel motors to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the flywheels at the specified percent output.
     */
    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            rm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
            lm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            rm.set(0);
            lm.set(0);
        });
    }

    /**
     * Sets the flywheels to the velocity specified by the tunable NetworkTable value.
     * Useful for tuning RPM without redeploying code.
     *
     * @return A Command that runs the flywheels at the tunable velocity.
     */
    public Command setVelocityTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
        }, () -> {
            rm.set(0);
            lm.set(0);
        });
    }

    /**
     * Sets the target velocity for the flywheels using a dynamic supplier (useful for auto-aiming).
     *
     * @param velo A DoubleSupplier providing the target RPM.
     * @return A Command that continually sets the RPM based on the supplier.
     */
    public Command setVelocity(DoubleSupplier velo) {
        return runOnce(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velo.getAsDouble())));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velo.getAsDouble())));
        });
    }

    /**
     * Command to immediately stop the flywheels by setting motor output to 0.
     *
     * @return Command that halts the flywheels.
     */
    public Command stop() {
        return runOnce(() -> {
            rm.set(0);
            lm.set(0);
        });
    }

    /**
     * Sets the flywheels to the percent output specified by the tunable NetworkTable value.
     * Useful for tuning voltage feedforward without redeploying code.
     *
     * @return A Command that runs the flywheels at the tunable percent output.
     */
    public Command setPercentOutTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
            lm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            rm.set(0);
            lm.set(0);
        });
    }

    /**
     * Checks if all flywheel motors are within the configured velocity tolerance of the target.
     *
     * @param targetVelo The desired target velocity.
     * @return true if all motors are near the target velocity.
     */
    public boolean isVelocityWithinTolerance(AngularVelocity targetVelo) {

    boolean rmOk = rmVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean rfOk = rfVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean lmOk = lmVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean lfOk = lfVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);

    return rmOk && rfOk && lmOk && lfOk;
    }

    /**
     * Fast check to see if all flywheels have met a base firing RPM threshold.
     *
     * @return true if all flywheels are securely over 2500 RPM.
     */
    public boolean velocityThresholdsMet() {
    boolean rmOk = (rmVelocity.getValue().gte(Units.RPM.of(2500)));
    boolean rfOk = (rfVelocity.getValue().gte(Units.RPM.of(2500)));
    boolean lmOk = (lmVelocity.getValue().gte(Units.RPM.of(2500)));
    boolean lfOk = (lfVelocity.getValue().gte(Units.RPM.of(2500)));

    return rmOk && rfOk && lmOk && lfOk;
    }

@Override
public void periodic() {
    sTunableRpm = shooterRpmTunable.get();
    sTunablePercentOut = shooterPercentOutTunable.get();

    BaseStatusSignal.refreshAll(
        rmVelocity, rfVelocity, lmVelocity, lfVelocity,
        rmCurrent, lmCurrent,
        rmTemp, rfTemp, lmTemp, lfTemp
    );

    // ── Velocity ──────────────────────────────────────────────────
    DogLog.log("Shooter/RightMaster/VelocityRPM",
        Units.RotationsPerSecond.of(rmVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/RightFollower/VelocityRPM",
        Units.RotationsPerSecond.of(rfVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/LeftMaster/VelocityRPM",
        Units.RotationsPerSecond.of(lmVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/LeftFollower/VelocityRPM",
        Units.RotationsPerSecond.of(lfVelocity.getValueAsDouble()).in(Units.RPM));

    // ── Current ───────────────────────────────────────────────────
    DogLog.log("Shooter/RightMaster/AppliedCurrent", rmCurrent.getValueAsDouble());
    DogLog.log("Shooter/LeftMaster/AppliedCurrent",  lmCurrent.getValueAsDouble());

    // ── Temperature ───────────────────────────────────────────────
    DogLog.log("Shooter/RightMaster/Temperature",   rmTemp.getValueAsDouble());
    DogLog.log("Shooter/RightFollower/Temperature", rfTemp.getValueAsDouble());
    DogLog.log("Shooter/LeftMaster/Temperature",    lmTemp.getValueAsDouble());
    DogLog.log("Shooter/LeftFollower/Temperature",  lfTemp.getValueAsDouble());

    // ── Connection ────────────────────────────────────────────────
    boolean rmConnected = rm.isConnected(2.0);
    boolean rfConnected = rf.isConnected(2.0);
    boolean lmConnected = lm.isConnected(2.0);
    boolean lfConnected = lf.isConnected(2.0);
    boolean allConnected = rmConnected && rfConnected && lmConnected && lfConnected;

    DogLog.log("Shooter/RightMaster/Connected",   rmConnected);
    DogLog.log("Shooter/RightFollower/Connected", rfConnected);
    DogLog.log("Shooter/LeftMaster/Connected",    lmConnected);
    DogLog.log("Shooter/LeftFollower/Connected",  lfConnected);

    AlertType alertType = DriverStation.isFMSAttached() ? null : AlertType.kError;

    if (!allConnected) { DogLog.logFault("CAN2: Shooter Disconnected",   alertType); }
    else               { DogLog.clearFault("CAN2: Shooter Disconnected"); }

    // ── Temperature faults ────────────────────────────────────────
}
}
