package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class Flywheel extends SubsystemBase {

    private TalonFX rm;
    private TalonFX rf;
    private TalonFX lm;
    private TalonFX lf;

    private final IntegerSubscriber shooterRpmTunable = DogLog.tunable("Shooter/TunableShooterVelocity", 2000);
    private final DoubleSubscriber shooterPercentOutTunable = DogLog.tunable("Shooter/TunableShooterOutput", 0.1);
    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Shooter/TunableShooterOutput", 0.1);

    double sTunableRpm = shooterRpmTunable.get();
    double sTunablePercentOut = shooterPercentOutTunable.get(); 
    double sTunableCowlPosition = cowlPositionTunable.get(); 

    VoltageOut flywheelVoltageOut = new VoltageOut(0);
    VelocityVoltage flywheelVelocityOut = new VelocityVoltage(0).withSlot(0);

    public Flywheel() {
        rm = new TalonFX(Ports.Flywheel.kRightFlywheelMaster);
        rf = new TalonFX(Ports.Flywheel.kRightFlywheelFollower);
        lm = new TalonFX(Ports.Flywheel.kLeftFlywheelMaster);
        lf = new TalonFX(Ports.Flywheel.kLeftFlywheelFollower);

        rm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lm.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        lf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);
        rf.getConfigurator().apply(SystemConstants.Flywheel.masterConfig);

        rf.setControl(new Follower(Ports.Flywheel.kRightFlywheelMaster, MotorAlignmentValue.Opposed));
        lf.setControl(new Follower(Ports.Flywheel.kRightFlywheelMaster, MotorAlignmentValue.Opposed));
    }


    public Command setVelocity(double velocityRPM) {
        return runEnd(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
        }, () -> {
            rm.set(0);
            rf.set(0);
        });
    }

    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            rm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
            lm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            rm.set(0);
            rf.set(0);
        });
    }

    public Command setVelocityTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
            lm.setControl(flywheelVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
        }, () -> {
            rm.set(0);
            rf.set(0);
        });
    }

    public Command setPercentOutTunable() {
        return runEnd(() -> {
            rm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
            lm.setControl(flywheelVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            rm.set(0);
            rf.set(0);
        });
    }

    @Override
    public void periodic() {
        // Update tunables
        sTunableRpm = shooterRpmTunable.get();
        sTunablePercentOut = shooterPercentOutTunable.get();
        sTunableCowlPosition = cowlPositionTunable.get();

        // Logging
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
    }
}
