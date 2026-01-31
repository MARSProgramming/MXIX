package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class Feeder extends SubsystemBase {

    private TalonFX m_feeder;

    private final IntegerSubscriber feederRpmTunable = DogLog.tunable("Feeder/TunableFeederVelocity", 2000);

    double sTunableRpm = feederRpmTunable.get();

    VelocityVoltage feederVelocityOut = new VelocityVoltage(0).withSlot(0);

    public Feeder() {
        m_feeder = new TalonFX(Ports.Floor.kFloorRollers);

        m_feeder.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }

    public Command setVelocity(double velocityRPM) {
        return runEnd(() -> {
            m_feeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(velocityRPM)));
        }, () -> {
            m_feeder.set(0);
        });
    }

    public Command setVelocityTunable() {
        return runEnd(() -> {
            m_feeder.setControl(feederVelocityOut.withVelocity(Units.RPM.of(sTunableRpm)));
        }, () -> {
            m_feeder.set(0);
        });
    }

    @Override
    public void periodic() {
        sTunableRpm = feederRpmTunable.get();

        DogLog.log("Feeder/TunableFeederVelocity",
                Units.RotationsPerSecond.of(m_feeder.getVelocity().getValueAsDouble()).in(Units.RPM));
    }
}
