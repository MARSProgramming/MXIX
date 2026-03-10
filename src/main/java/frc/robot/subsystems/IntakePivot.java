package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.constants.Settings.IntakePivotSettings;
import frc.robot.constants.SystemConstants;

/**
 * Subsystem representing the Cowl mechanism.
 * This subsystem controls the position of the cowl using a TalonFX motor.
 */
public class IntakePivot extends SubsystemBase {
    TalonFX mIntakePivot;

    // Tunable value for testing position setpoints via NetworkTables
    private final DoubleSubscriber pivotPercentOutTunable = DogLog.tunable("IntakePivot/TunablePercentOut", 0.5);
    double cTunablePivotOut = 0;
    // Rate limiter for periodic logging
    private int logCounter = 0;
    
    VoltageOut floorVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public IntakePivot() {
        mIntakePivot = new TalonFX(Ports.Intake.kIntakePivot);
        mIntakePivot.getConfigurator().apply(SystemConstants.Intake.pivotConfig);
    }


    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(percentOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    public Command deployCommand() {
        return runEnd(
            () -> {
                mIntakePivot.setControl(floorVoltageOut.withOutput(Settings.IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE * 12));
            },
            () -> {
                mIntakePivot.set(0);
            }
        );
    }

    public Command retractCommand() {
        return runEnd(
            () -> {
                mIntakePivot.setControl(floorVoltageOut.withOutput(-Settings.IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE * 12));
            },
            () -> {
                mIntakePivot.set(0);
            }
        );
    }


    public Command forwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(cTunablePivotOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

        public Command backwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(floorVoltageOut.withOutput(-cTunablePivotOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    
    public Command slamtake() {
    // assumes intake is deployed. 
    return Commands.repeatingSequence(
        this.setPercentOut(-IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT),
        Commands.waitSeconds(1),
        this.setPercentOut(IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_RETRACT_TIMEOUT),
        Commands.waitSeconds(1)
        );
    }

    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
        cTunablePivotOut = 0;

        // Log current position
        if (++logCounter >= 5) {
            logCounter = 0;
            DogLog.log("Intake/Pivot/Position", mIntakePivot.getPosition().getValueAsDouble());
            DogLog.log("Intake/Pivot/AppliedVoltage", mIntakePivot.getMotorVoltage().getValueAsDouble());
            DogLog.log("Intake/Pivot/Temperature", mIntakePivot.getDeviceTemp().getValueAsDouble());
        }
    }
}