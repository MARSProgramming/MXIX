package frc.robot.constants;


import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

/** CLAUDE SUGGESTION
 * Contains system-wide constants and hardware configurations for subsystems.
 * This includes physical constants (gear ratios, max speeds) and CTRE Phoenix 6 configurations.
 */
public class SystemConstants {

    public static final double kTalonFXWarnTemp = 75.0;
    /**
     * Drivetrain constants not handled by Tuner X generation.
     */
    public static class Drive {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = Units.RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class Limelights {
        public static final String kShooterLimelightName = "limelight-shooter";
        public static final String kClimbLimelightName = "limelight-back";

        public static int[] getValidTagIDs() {
            return new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32}; // Valid
            // climb tags: 15, 16, 31, 32
        }

        // we don't need to change based on alliance, the filtering is useless.
    }

    /**
     * Constants for the Kraken X60 motor.
     */
    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = Units.RPM.of(6000);
    }

    /**
     * Constants for the Kraken X44 motor.
     */
    public static class KrakenX44 {
        public static final AngularVelocity kFreeSpeed = Units.RPM.of(7530);
    }

    /**
     * Constants and configuration for the Flywheel (Shooter) subsystem.
     */
    public static class Flywheel {
        public static final Transform3d ROBOT_TO_SHOOTER_TRANSFORM = new Transform3d(
                new Translation3d(-0.044704, -0.055626, 0.525526),
                new Rotation3d(0, 0, 0)
        );        

        public static final double kFlywheelReduction = 1.0;
        public static final AngularVelocity kVelocityTolerance = Units.RPM.of(200);
        public static final AngularVelocity kMaxFlywheelSpeed = KrakenX60.kFreeSpeed.div(kFlywheelReduction);

        public static TalonFXConfiguration masterConfig = new TalonFXConfiguration();        
        static {
            // Velocity PID constants
            masterConfig.Slot0.kP = 0.5;
            masterConfig.Slot0.kI = 2;
            masterConfig.Slot0.kD = 0;
            masterConfig.Slot0.kV = 12.0 / kMaxFlywheelSpeed.in(Units.RotationsPerSecond);

            // Motor output configuration
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            masterConfig.Feedback.SensorToMechanismRatio = kFlywheelReduction; 

            // Current limits
            masterConfig.Voltage.PeakReverseVoltage = 0; // Software lock reversal of flywheel 
            masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            masterConfig.CurrentLimits.SupplyCurrentLimit = 70;
            
            masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            masterConfig.CurrentLimits.StatorCurrentLimit = 120;
        }
    }

    /**
     * Constants and configuration for the Cowl subsystem.
     */
    public static class Cowl {
        public static final double kCowlStallCurrent = 7.0; // Configure with testing
        public static final double kCowlStallTimeout = 0.47; // 600 ms
        public static final double kCowlHomingOutput = -0.3; // Configure with testing, call percentOut to home

        public static TalonFXConfiguration cowlConfig = new TalonFXConfiguration();
        static {
            // Position Control pid (requires tuning)

            cowlConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cowlConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            cowlConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            cowlConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.80;
            cowlConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            cowlConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1;

            cowlConfig.Slot0.kP = 20;
            cowlConfig.Slot0.kI = 0;
            cowlConfig.Slot0.kD = 0;
            cowlConfig.Slot0.kA = 0;
            cowlConfig.Slot0.kV = 0;
            cowlConfig.Slot0.kG = 0;
            cowlConfig.Slot0.kS = 2;


            cowlConfig.Feedback.SensorToMechanismRatio = 1.0; // Gear ratio

            cowlConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            cowlConfig.CurrentLimits.SupplyCurrentLimit = 30;
            cowlConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            cowlConfig.CurrentLimits.StatorCurrentLimit = 50;
        }
    }
    
    /**
     * Constants and configuration for the Intake subsystem (Pivot and Rollers).
     */
    public static class Intake {
        public static final double kIntakePivotReduction = 4 / 1; // Same for both pivot and rollers
        public static final double kIntakeRollerReduction = 1.0; // Same for both pivot and rollers

        private static final AngularVelocity kMaxIntakePivotSpeed = KrakenX44.kFreeSpeed.div(kIntakePivotReduction); // same for both pivot and rollers
        private static final AngularVelocity kMaxIntakeRollerSpeed = KrakenX44.kFreeSpeed.div(kIntakeRollerReduction); // same for both pivot and rollers
        public static final double kIntakePivotStallCurrent = 1; // Configure with testing, this is way too low

        public static TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        public static TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        static {
            // Pivot Position Control PID (requires tuning)
            pivotConfig.Slot0.kP = 1;
            pivotConfig.Slot0.kI = 0;
            pivotConfig.Slot0.kD = 0;
            pivotConfig.Slot0.kV = 12.0 / kMaxIntakePivotSpeed.in(Units.RotationsPerSecond);

            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            pivotConfig.Feedback.SensorToMechanismRatio = kIntakePivotReduction; // Gear ratio

            pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
            pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            pivotConfig.CurrentLimits.StatorCurrentLimit = 80;
        }

        static {
            // Roller Velocity control PID (if needed)
            rollerConfig.Slot0.kP = 0.5;
            rollerConfig.Slot0.kI = 2;
            rollerConfig.Slot0.kD = 0;
            rollerConfig.Slot0.kV = 12.0 / kMaxIntakeRollerSpeed.in(Units.RotationsPerSecond);

            rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            rollerConfig.Feedback.SensorToMechanismRatio = kIntakeRollerReduction; 

            rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.SupplyCurrentLimit = 60;
            rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.StatorCurrentLimit = 100;
        }
    }

    /**
     * Constants and configuration for the Floor Intake subsystem.
     */
    public static class Floor {
        public static final double kFloorReduction = 1.0;
        private static final AngularVelocity kMaxFloorSpeed = KrakenX44.kFreeSpeed.div(kFloorReduction);

        public static TalonFXConfiguration floorConfig = new TalonFXConfiguration();

        static {
            // Velocity control (if needed)
            floorConfig.Slot0.kP = 0.5;
            floorConfig.Slot0.kI = 2;
            floorConfig.Slot0.kD = 0;
            floorConfig.Slot0.kV = 12.0 / kMaxFloorSpeed.in(Units.RotationsPerSecond);

            floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            floorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            floorConfig.Feedback.SensorToMechanismRatio = kFloorReduction; 

            floorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            floorConfig.CurrentLimits.SupplyCurrentLimit = 60;
            floorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            floorConfig.CurrentLimits.StatorCurrentLimit = 100;
        }
    }

    
    /**
     * Constants and configuration for the Feeder subsystem.
     */
    public static class Feeder {
        public static final double kFeederReduction = 4 / 1; // 4:1 Sport gearbox transmission
        private static final AngularVelocity kMaxFeederSpeed = KrakenX60.kFreeSpeed.div(kFeederReduction);
        public static final AngularVelocity kVelocityTolerance = Units.RPM.of(100);
        public static TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        static {
            // Velocity control (if needed)
            feederConfig.Slot0.kP = 1;
            feederConfig.Slot0.kI = 0;
            feederConfig.Slot0.kD = 0;
            feederConfig.Slot0.kV = 12.0 / kMaxFeederSpeed.in(Units.RotationsPerSecond);

            feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            feederConfig.Feedback.SensorToMechanismRatio = kFeederReduction; 

            feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
            feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            feederConfig.CurrentLimits.StatorCurrentLimit = 100;
        }
    }

    
    /**
     * Constants and configuration for the Fast Climber subsystem.
     */
    public static class FastClimber {
        public static final double kFastClimberReduction = 80 / 1;
        private static final AngularVelocity kMaxFastClimberSpeed = KrakenX60.kFreeSpeed.div(kFastClimberReduction);
        
        public static TalonFXConfiguration fastClimberConfig = new TalonFXConfiguration();

        static {
            // position control (if needed)
            fastClimberConfig.Slot0.kP = 1; // definitely too low if we need position control
            fastClimberConfig.Slot0.kI = 0;
            fastClimberConfig.Slot0.kD = 0;
            fastClimberConfig.Slot0.kV = 12.0 / kMaxFastClimberSpeed.in(Units.RotationsPerSecond);

            fastClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            fastClimberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            fastClimberConfig.Feedback.SensorToMechanismRatio = kFastClimberReduction; 

            fastClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            fastClimberConfig.CurrentLimits.SupplyCurrentLimit = 70;
            fastClimberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            fastClimberConfig.CurrentLimits.StatorCurrentLimit = 120;

            fastClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            fastClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.62;
            fastClimberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        }
    }
}
