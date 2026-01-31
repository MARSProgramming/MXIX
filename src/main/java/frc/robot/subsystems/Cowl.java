package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class Cowl extends SubsystemBase {
    TalonFX m_cowl;

    PositionVoltage cowlPositionOut = new PositionVoltage(0);

    private final DoubleSubscriber cowlPositionTunable = DogLog.tunable("Cowl/TunableCowlOutput", 0.1);
    double cTunablePosition = cowlPositionTunable.get();

    public Cowl() {
        m_cowl = new TalonFX(Ports.Cowl.kCowlMotor);
        m_cowl.getConfigurator().apply(SystemConstants.Cowl.cowlConfig);

    }

    public Command setPosition(double position) {
        return runEnd(() -> {
            m_cowl.setControl(cowlPositionOut.withPosition(position));
        }, () -> {
            m_cowl.set(0);
        });
    }

    public Command setPositionTunable() {
        return runEnd(() -> {
            m_cowl.setControl(cowlPositionOut.withPosition(cTunablePosition));
        }, () -> {
            m_cowl.set(0);
        });
    }

    @Override
    public void periodic() {
        cTunablePosition = cowlPositionTunable.get();

        DogLog.log("Cowl/Position", m_cowl.getPosition().getValueAsDouble());
    }
}
