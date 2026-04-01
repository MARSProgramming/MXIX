package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * Subsystem representing the Intake Pivot mechanism.
 * This subsystem controls the deployment and retraction of the intake arm.
 */
/**
 * Subsystem: IntakePivot
 * Responsible for controlling the IntakePivot mechanism.
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
     * Creates a new Intake Pivot subsystem.
     * Initializes the pivot motor and applies the system configuration.
     */
    public IntakePivot() {
        mIntakePivot = new TalonFX(Ports.Intake.kIntakePivot, new CANBus("CAN2"));
        mIntakePivot.getConfigurator().apply(SystemConstants.Intake.pivotConfig);

        mIntakePivot.optimizeBusUtilization();

        mIntakePivot.getPosition().setUpdateFrequency(50);
        mIntakePivot.getMotorVoltage().setUpdateFrequency(10);
        mIntakePivot.getDeviceTemp().setUpdateFrequency(4);

        mPosition = mIntakePivot.getPosition();
        mVoltage = mIntakePivot.getMotorVoltage();
        mTemp = mIntakePivot.getDeviceTemp();
    }


    /**
     * Returns a command indicating voltage percentage output for the pivot.
     * 
     * @param percentOut Output duty cycle percentage (-1.0 to 1.0).
     * @return A Command running the pivot at raw duty cycle.
     */
    public Command setPercentOut(double percentOut) {
        return runEnd(() -> {
            mIntakePivot.setControl(pivotVoltageOut.withOutput(percentOut * 12.0));
        }, () -> {
            mIntakePivot.set(0);
        });
    }

    /**
     * Deploys the intake pivot using a configured voltage duty cycle.
     * Runs continuously until interrupted or stopped.
     *
     * @return Command to deploy intake.
     */
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

    /**
     * Deploys the intake pivot using a configured timeout.
     * Will stop the intake automatically after the configured duration.
     *
     * @return Command to deploy intake for a fixed time.
     */
    public Command timedDeployCommand() {
        return deployCommand().withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT);
    }

    /**
     * Retracts the intake pivot using a configured inverse voltage duty cycle.
     * Runs continuously until interrupted or stopped.
     *
     * @return Command to retract intake.
     */
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

    
    /**
     * Performs a 'slamtake' maneuver to ensure a game piece is fully captured.
     * This involves an alternating retract/deploy sequence.
     * Assumes intake is already deployed.
     *
     * @return A sequence of commands executing the slamtake behavior.
     */
    public Command slamtake() {
    // assumes intake is deployed. 
    return Commands.repeatingSequence(
        this.setPercentOut(-IntakePivotSettings.INTAKE_LIFT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_RETRACT_TIMEOUT),
        Commands.waitSeconds(0.2),
        this.setPercentOut(IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(Settings.IntakePivotSettings.INTAKE_DEPLOY_TIMEOUT),
        Commands.waitSeconds(0.2)
        );
    }

    /**
     * Momentarily drives the intake outwards to confirm deployment depth.
     *
     * @return Command to briefly jog the intake out.
     */
    public Command confirmDeploy() {
    return Commands.sequence(
        this.setPercentOut(IntakePivotSettings.INTAKE_DEPLOYMENT_DUTYCYCLE).withTimeout(0.1)
        );
    }


@Override
public void periodic() {
    cTunablePivotOut = pivotPercentOutTunable.get();

    BaseStatusSignal.refreshAll(mPosition, mVoltage, mTemp);

    boolean connected = mIntakePivot.isConnected(2.0);

    DogLog.log("IntakePivot/Position",       mPosition.getValueAsDouble());
    DogLog.log("IntakePivot/AppliedVoltage", mVoltage.getValueAsDouble());
    DogLog.log("IntakePivot/Temperature",    mTemp.getValueAsDouble());
    DogLog.log("IntakePivot/Connected",      connected);

    AlertType alertType = DriverStation.isFMSAttached() ? null : AlertType.kError;

    if (!connected) { DogLog.logFault("CAN2: IntakePivot Disconnected",   alertType); }
    else            { DogLog.clearFault("CAN2: IntakePivot Disconnected"); }

    }
}