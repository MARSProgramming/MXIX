package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class Feeder extends SubsystemBase {

    private TalonFX mFeeder;

    private final IntegerSubscriber feederRpmTunable = DogLog.tunable("Feeder/TunableFeederVelocity", 2000);

    double sTunableRpm = feederRpmTunable.get();

    VelocityVoltage feederVelocityOut = new VelocityVoltage(0).withSlot(0);
    VoltageOut feederVoltageOut = new VoltageOut(0);

    public Feeder() {
        mFeeder = new TalonFX(Ports.Feeder.kFeederMotor);
        mFeeder.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }

    public Command setVelocity(double velocityRPM) {
        return runEnd(() -> {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    public Command setVelocityTunable() {
        return runEnd(() -> {
            mFeeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    public Command set(double percentOut) {
        return runEnd(() -> {
            mFeeder.setControl(feederVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            mFeeder.set(0);
        });
    }

    @Override
    public void periodic() {
        sTunableRpm = feederRpmTunable.get();
        DogLog.log("Feeder/VelocityRPM", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
        DogLog.log("Feeder/AppliedVoltage", mFeeder.getMotorVoltage().getValueAsDouble());
        DogLog.log("Feeder/Temperature", mFeeder.getDeviceTemp().getValueAsDouble());
        DogLog.log("Feeder/TunableFeederVelocity", Units.RotationsPerSecond.of(mFeeder.getVelocity().getValueAsDouble()).in(Units.RPM));
    }

    public boolean isVelocityWithinTolerance() {
        final boolean isInVelocityMode = mFeeder.getAppliedControl().equals(feederVelocityOut);
        final AngularVelocity currentVelocity = mFeeder.getVelocity().getValue();
        final AngularVelocity targetVelocity = feederVelocityOut.getVelocityMeasure();
        return isInVelocityMode && currentVelocity.isNear(targetVelocity, SystemConstants.Feeder.kVelocityTolerance);
    }
}
