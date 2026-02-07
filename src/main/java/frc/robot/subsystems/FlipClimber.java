package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Cowl mechanism.
 * This subsystem controls the position of the cowl using a TalonFX motor.
 */
public class FlipClimber extends SubsystemBase {
    TalonFX mFlipClimbMaster;
    TalonFX mFlipClimbFollower;

    // Control request for position control using voltage
    PositionVoltage flipPositionOut = new PositionVoltage(0);

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber flipClimbVoltageTunable = DogLog.tunable("FlipClimber/TunableClimbOutput", 0.1);
    double cTunableOutput = flipClimbVoltageTunable.get();

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public FlipClimber() {
        mFlipClimbMaster = new TalonFX(Ports.FlipClimber.kClimbMaster);
        mFlipClimbFollower = new TalonFX(Ports.FlipClimber.kClimbFollower);

        mFlipClimbMaster.getConfigurator().apply(SystemConstants.FlipClimber.flipClimberConfig);
        mFlipClimbFollower.getConfigurator().apply(SystemConstants.FlipClimber.flipClimberConfig);

        mFlipClimbFollower.setControl(new Follower(Ports.FlipClimber.kClimbMaster, MotorAlignmentValue.Opposed));

    }


    public Command setPercentOut(double output) {
        return runEnd(() -> {
            mFlipClimbMaster.set(output);
        }, () -> {
            mFlipClimbMaster.set(0);
        });
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunableOutput = flipClimbVoltageTunable.get();

        // Log current position
        DogLog.log("FlipClimber/Master/Position", mFlipClimbMaster.getPosition().getValueAsDouble());
        DogLog.log("FlipClimber/Master/AppliedVoltage", mFlipClimbMaster.getMotorVoltage().getValueAsDouble());
        DogLog.log("FlipClimber/Master/Temperature", mFlipClimbMaster.getDeviceTemp().getValueAsDouble());

        DogLog.log("FlipClimber/Follower/Position", mFlipClimbFollower.getPosition().getValueAsDouble());
        DogLog.log("FlipClimber/Follower/AppliedVoltage", mFlipClimbFollower.getMotorVoltage().getValueAsDouble());
        DogLog.log("FlipClimber/Follower/Temperature", mFlipClimbFollower.getDeviceTemp().getValueAsDouble());

        DogLog.log("FlipClimber/TunableVoltage", cTunableOutput);

    }
}
