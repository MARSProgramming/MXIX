package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Shooter mechanism, which includes the Flywheel and Cowl.
 * Controls the four motors for the flywheel (2 masters, 2 followers) used to launch game pieces,
 * and one motor for positioning the cowl.
 */
public class Shooter extends SubsystemBase {

    // Flywheel motors
    private TalonFX rm;
    private TalonFX rf;
    private TalonFX lm;
    private TalonFX lf;
    private final List<TalonFX> motors;

    // Cowl motor
    TalonFX m_cowl;

    // Tunable values for flywheel
    private final IntegerSubscriber shooterRpmTunable = DogLog.tunable("Shooter/TunableShooterVelocity", 2000);
    private final DoubleSubscriber shooterPercentOutTunable = DogLog.tunable("Shooter/TunableShooterOutput", 0.1);
    double sTunableRpm = shooterRpmTunable.get();
    double sTunablePercentOut = shooterPercentOutTunable.get(); 

    // Tunable value for cowl
    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Shooter/Cowl/TunableCowlOutput", 0.1);
    double cTunablePosition = cowlPositionTunable.get();

    // Control requests for flywheel
    VoltageOut flywheelVoltageOut = new VoltageOut(0);
    VelocityVoltage flywheelVelocityOut = new VelocityVoltage(0).withSlot(0);

    // Control request for cowl
    PositionVoltage cowlPositionOut = new PositionVoltage(0);

    /**
     * Creates a new Shooter subsystem.
     * Initializes flywheel and cowl motors, applies configurations, and sets up follower relationships for the flywheel.
     */
    public Shooter() {
        // Flywheel motor initialization
        rm = new TalonFX(Ports.Flywheel.kRightFlywheelMaster);
        rf = new TalonFX(Ports.Flywheel.kRightFlywheelFollower);
        lm = new TalonFX(Ports.Flywheel.kLeftFlywheelMaster);
        lf = new TalonFX(Ports.Flywheel.kLeftFlywheelFollower);

        // Apply flywheel motor configurations
        rm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        rf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);

        // Configure followers to oppose the masters (since they are on opposite sides/gearing)
        rf.setControl(new Follower(Ports.Flywheel.kRightFlywheelMaster, MotorAlignmentValue.Opposed));
        lf.setControl(new Follower(Ports.Flywheel.kLeftFlywheelMaster, MotorAlignmentValue.Opposed));

        motors = List.of(rm, rf, lm, lf);

        // Cowl motor initialization and configuration
        m_cowl = new TalonFX(Ports.Cowl.kCowlMotor);
        m_cowl.getConfigurator().apply(SystemConstants.Cowl.cowlConfig);

    }

    /**
     * Sets the target velocity for the flywheels in RPM.
     * This method sets the control request but does not manage the command lifecycle.
     *
     * @param velocityRPM The target velocity in Rotations Per Minute.
     */
    public void setShooterRPM(double velocityRPM) {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
    }

    /**
     * Command to spin up the flywheels and wait until they reach the target velocity.
     * Useful in autonomous modes to ensure the shooter is ready before feeding.
     *
     * @param velocityRPM The target velocity in RPM.
     * @return A Command that sets the RPM and waits for the flywheel to be within tolerance.
     */
    public Command spinUpShooter(double velocityRPM) {
        return runOnce(() -> setShooterRPM(velocityRPM))
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    /**
     * Sets the flywheel motors to a percentage of output voltage.
     *
     * @param percentOut The percentage of output (0.0 to 1.0).
     * @return A Command that runs the flywheels at the specified percent output.
     */
    public Command setShooterPercentOut(double percentOut) {
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
    public Command setShooterVelocityTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
        }, () -> {
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
    public Command setShooterPercentOutTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
            lm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            rm.set(0);
            lm.set(0);
        });
    }

    /**
     * Sets the cowl to a specific position.
     * The command runs until interrupted or finished, stopping the motor on end.
     *
     * @param position The target position in rotations.
     * @return A Command that moves the cowl to the specified position.
     */
    public Command setCowlPosition(double position) {
        return runEnd(() -> {
            m_cowl.setControl(cowlPositionOut.withPosition(position));
        }, () -> {
            m_cowl.set(0);
        });
    }

    /**
     * Sets the cowl to the position specified by the tunable NetworkTable value.
     * Useful for tuning the position setpoint without redeploying code.
     *
     * @return A Command that moves the cowl to the tunable position.
     */
    public Command setCowlPositionTunable() {
        return runEnd(() -> {
            m_cowl.setControl(cowlPositionOut.withPosition(cTunablePosition));
        }, () -> {
            m_cowl.set(0);
        });
    }

    /**
     * Checks if all flywheel motors are within the configured velocity tolerance of the target.
     *
     * @return true if all motors are in velocity mode and near the target velocity.
     */
    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(flywheelVelocityOut);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = flywheelVelocityOut.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, SystemConstants.Flywheel.kVelocityTolerance);
        });
    }


    @Override
    public void periodic() {
        // Update local tunable variables from NetworkTables
        sTunableRpm = shooterRpmTunable.get();
        sTunablePercentOut = shooterPercentOutTunable.get();
        cTunablePosition = cowlPositionTunable.get();

        // Flywheel Logging
        DogLog.log("Shooter/RightMaster/VelocityRPM", Units.RotationsPerSecond.of(rm.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Shooter/RightFollower/VelocityRPM", Units.RotationsPerSecond.of(rf.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Shooter/LeftMaster/VelocityRPM", Units.RotationsPerSecond.of(lm.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Shooter/LeftFollower/VelocityRPM", Units.RotationsPerSecond.of(lf.getVelocity().getValueAsDouble()).in(Units.RPM));

        DogLog.log("Shooter/RightMaster/AppliedVoltage", rm.getMotorVoltage().getValueAsDouble());
        DogLog.log("Shooter/LeftMaster/AppliedVoltage", lm.getMotorVoltage().getValueAsDouble());

        DogLog.log("Shooter/RightMaster/Temperature", rm.getDeviceTemp().getValueAsDouble());
        DogLog.log("Shooter/RightFollower/Temperature", rf.getDeviceTemp().getValueAsDouble());
        DogLog.log("Shooter/LeftMaster/Temperature", rm.getDeviceTemp().getValueAsDouble());
        DogLog.log("Shooter/LeftFollower/Temperature", rf.getDeviceTemp().getValueAsDouble());

        // Cowl Logging
        DogLog.log("Shooter/Colw/Position", m_cowl.getPosition().getValueAsDouble());
    }
}
