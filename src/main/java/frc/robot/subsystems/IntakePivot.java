package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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
    double cTunablePivotOut = pivotPercentOutTunable.get(); 
    
    private final StatusSignal<Angle> mPosition;
    private final StatusSignal<Voltage> mVoltage;
    private final StatusSignal<Temperature> mTemp;

    VoltageOut pivotVoltageOut = new VoltageOut(0);

    /**
     * Creates a new Cowl subsystem.
     * Initializes the motor and applies the configuration.
     */
    public IntakePivot() {
        mIntakePivot = new TalonFX(Ports.Intake.kIntakePivot);
        mIntakePivot.getConfigurator().apply(SystemConstants.Intake.pivotConfig);

        mIntakePivot.optimizeBusUtilization();

        mIntakePivot.getPosition().setUpdateFrequency(50);
        mIntakePivot.getMotorVoltage().setUpdateFrequency(10);
        mIntakePivot.getDeviceTemp().setUpdateFrequency(4);

        mPosition = mIntakePivot.getPosition();
        mVoltage = mIntakePivot.getMotorVoltage();
        mTemp = mIntakePivot.getDeviceTemp();
    }


    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            mIntakePivot.setControl(pivotVoltageOut.withOutput(percentOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    public Command deployCommand() {
        return runEnd(
            () -> {
                mIntakePivot.setControl(pivotVoltageOut.withOutput(Settings.IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE * 12));
            },
            () -> {
                mIntakePivot.set(0);
            }
        );
    }

    public Command timedDeployCommand() {
        return deployCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT);
    }

    public Command retractCommand() {
        return runEnd(
            () -> {
                mIntakePivot.setControl(pivotVoltageOut.withOutput(-Settings.IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE * 12));
            },
            () -> {
                mIntakePivot.set(0);
            }
        );
    }


    public Command forwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(pivotVoltageOut.withOutput(cTunablePivotOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

        public Command backwardTunable() {
        return runEnd(() -> {
            mIntakePivot.setControl(pivotVoltageOut.withOutput(-cTunablePivotOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    
    public Command slamtake() {
    // assumes intake is deployed. 
    return Commands.repeatingSequence(
        this.setPercentOut(-IntakePivotSettings.INTAKE_LIFT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_RETRACT_TIMEOUT),
        Commands.waitSeconds(0.2),
        this.setPercentOut(IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT),
        Commands.waitSeconds(0.2)
        );
    }

    public Command confirmDeploy() {
    return Commands.sequence(
        this.setPercentOut(IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(0.1)
        );
    }


    @Override
    public void periodic() {
        // Update local tunable variable from NetworkTables
    cTunablePivotOut = pivotPercentOutTunable.get(); // was being reset to 0 — bug fix

    BaseStatusSignal.refreshAll(mPosition, mVoltage, mTemp);

    DogLog.log("IntakePivot/Position",       mPosition.getValueAsDouble());
    DogLog.log("IntakePivot/AppliedVoltage", mVoltage.getValueAsDouble());
    DogLog.log("IntakePivot/Temperature",    mTemp.getValueAsDouble());
    }
}