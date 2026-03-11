package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Flywheel (Shooter) mechanism.
 * Controls the four motors (2 masters, 2 followers) used to launch game pieces.
 */
public class Flywheel extends SubsystemBase {

    private TalonFX rm;
    private TalonFX rf;
    private TalonFX lm;
    private TalonFX lf;
    private final List<TalonFX> motors;

    private SlewRateLimiter spinUpLimiter = new SlewRateLimiter(300);

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
        rm = new TalonFX(Ports.Flywheel.kRightFlywheelMaster);
        rf = new TalonFX(Ports.Flywheel.kRightFlywheelFollower);
        lm = new TalonFX(Ports.Flywheel.kLeftFlywheelMaster);
        lf = new TalonFX(Ports.Flywheel.kLeftFlywheelFollower);

        rm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        rf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);

        

        // Configure followers to oppose the masters (since they are on opposite sides/gearing)
        rf.setControl(new Follower(Ports.Flywheel.kRightFlywheelMaster, MotorAlignmentValue.Opposed));
        lf.setControl(new Follower(Ports.Flywheel.kLeftFlywheelMaster, MotorAlignmentValue.Opposed));

        motors = List.of(rm, rf, lm, lf);

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
     * Command to spin up the flywheels and wait until they reach the target velocity.
     * Useful in autonomous modes to ensure the shooter is ready before feeding.
     *
     * @param velocityRPM The target velocity in RPM.
     * @return A Command that sets the RPM and waits for the flywheel to be within tolerance.
     */
    public Command setVelocitySlew(double velocityRPM) {
        return run(() -> {
            double desiredRpms = spinUpLimiter.calculate(velocityRPM);
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
        })
        .until(() -> isVelocityWithinTolerance(getVelocity()));
    }

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

    public AngularVelocity getVelocity() {
        return rm.getVelocity().getValue();          
    }

    public void setSlewRPM(double velocityRPM) {
        double desiredRpms = spinUpLimiter.calculate(velocityRPM);
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

    public Command setVelocity(DoubleSupplier velo) {
        return runOnce(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velo.getAsDouble())));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velo.getAsDouble())));
        });
    }

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
     * @return true if all motors are in velocity mode and near the target velocity.
     */
    public boolean isVelocityWithinTolerance(AngularVelocity targetVelo) {

    boolean rmOk = rmVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean rfOk = rfVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean lmOk = lmVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);
    boolean lfOk = lfVelocity.getValue().isNear(targetVelo, SystemConstants.Flywheel.kVelocityTolerance);

    return rmOk && rfOk && lmOk && lfOk;
    }

    @Override
    public void periodic() {
        // Update local tunable variables from NetworkTables
        sTunableRpm = shooterRpmTunable.get();
        sTunablePercentOut = shooterPercentOutTunable.get();

    BaseStatusSignal.refreshAll(
        rmVelocity, rfVelocity, lmVelocity, lfVelocity,
        rmCurrent, lmCurrent,
        rmTemp, rfTemp, lmTemp, lfTemp
    );

    DogLog.log("Shooter/RightMaster/VelocityRPM",
        Units.RotationsPerSecond.of(rmVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/RightFollower/VelocityRPM",
        Units.RotationsPerSecond.of(rfVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/LeftMaster/VelocityRPM",
        Units.RotationsPerSecond.of(lmVelocity.getValueAsDouble()).in(Units.RPM));
    DogLog.log("Shooter/LeftFollower/VelocityRPM",
        Units.RotationsPerSecond.of(lfVelocity.getValueAsDouble()).in(Units.RPM));

    DogLog.log("Shooter/RightMaster/AppliedCurrent", rmCurrent.getValueAsDouble());
    DogLog.log("Shooter/LeftMaster/AppliedCurrent",  lmCurrent.getValueAsDouble());

    DogLog.log("Shooter/RightMaster/Temperature", rmTemp.getValueAsDouble());
    DogLog.log("Shooter/RightFollower/Temperature", rfTemp.getValueAsDouble());
    DogLog.log("Shooter/LeftMaster/Temperature",  lmTemp.getValueAsDouble()); 
    DogLog.log("Shooter/LeftFollower/Temperature", lfTemp.getValueAsDouble()); 
        
    }
}
