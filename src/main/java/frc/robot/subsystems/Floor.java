package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

public class Floor extends SubsystemBase {

    private TalonFX m_floor;

    private final DoubleSubscriber floorPercentOutTunable = DogLog.tunable("Floor/TunableFloorOutput", 0.1);

    double sTunablePercentOut = floorPercentOutTunable.get();

    VoltageOut floorVoltageOut = new VoltageOut(0);

    public Floor() {
        m_floor = new TalonFX(Ports.Floor.kFloorRollers);

        m_floor.getConfigurator().apply(SystemConstants.Floor.floorConfig);
    }

    public Command set(double percentOut) {
        return runEnd(() -> {
            m_floor.setControl(floorVoltageOut.withOutput(Units.Volts.of(percentOut * 12.0)));
        }, () -> {
            m_floor.set(0);
        });
    }

    public Command setPercentOutTunable() {
        return runEnd(() -> {
            m_floor.setControl(floorVoltageOut.withOutput(Units.Volts.of(sTunablePercentOut * 12.0)));
        }, () -> {
            m_floor.set(0);
        });
    }

    @Override
    public void periodic() {
        sTunablePercentOut = floorPercentOutTunable.get();

        DogLog.log("Floor/TunableFloorOutput", m_floor.getMotorVoltage().getValueAsDouble());
    }
}
