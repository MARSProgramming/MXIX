package frc.robot.constants;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * Contains system-wide constants and hardware configurations for subsystems.
 * This includes physical constants (gear ratios, max speeds) and CTRE Phoenix 6 configurations.
 */
public class SystemConstants {
    /**
     * Drivetrain constants not handled by Tuner X generation.
     */
    public static class Drive {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = Units.RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
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
        public static final double kFlywheelReduction = 1.0;
        public static final AngularVelocity kVelocityTolerance = Units.RPM.of(100);
        private static final AngularVelocity kMaxFlywheelSpeed = KrakenX60.kFreeSpeed.div(kFlywheelReduction);

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
        public static final double kCowlReduction = (24.0 / 12.0) * (260.0 / 17.0);
        private static final AngularVelocity kMaxCowlSpeed = KrakenX44.kFreeSpeed.div(kCowlReduction);
        public static final double kCowlStallCurrent = 1; // Configure with testing, this is way too low
        public static final double kCowlHomingOutput = -0.1; // Configure with testing, call percentOut to home

        public static TalonFXConfiguration cowlConfig = new TalonFXConfiguration();
        static {
            // Position Control pid (requires tuning)
            cowlConfig.Slot0.kP = 1;
            cowlConfig.Slot0.kI = 0;
            cowlConfig.Slot0.kD = 0;
            cowlConfig.Slot0.kV = 12.0 / kMaxCowlSpeed.in(Units.RotationsPerSecond);

            cowlConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            cowlConfig.Feedback.SensorToMechanismRatio = kCowlReduction; // Gear ratio

            cowlConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            cowlConfig.CurrentLimits.SupplyCurrentLimit = 70;
            cowlConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            cowlConfig.CurrentLimits.StatorCurrentLimit = 120;
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

            pivotConfig.Feedback.SensorToMechanismRatio = kIntakePivotReduction; // Gear ratio

            pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            pivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
            pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            pivotConfig.CurrentLimits.StatorCurrentLimit = 120;
        }

        static {
            // Roller Velocity control PID (if needed)
            rollerConfig.Slot0.kP = 0.5;
            rollerConfig.Slot0.kI = 2;
            rollerConfig.Slot0.kD = 0;
            rollerConfig.Slot0.kV = 12.0 / kMaxIntakeRollerSpeed.in(Units.RotationsPerSecond);

            rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            rollerConfig.Feedback.SensorToMechanismRatio = kIntakeRollerReduction; 

            rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.SupplyCurrentLimit = 70;
            rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            rollerConfig.CurrentLimits.StatorCurrentLimit = 120;
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
            floorConfig.CurrentLimits.SupplyCurrentLimit = 70;
            floorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            floorConfig.CurrentLimits.StatorCurrentLimit = 120;
        }
    }

    
    /**
     * Constants and configuration for the Feeder subsystem.
     */
    public static class Feeder {
        public static final double kFloorReduction = 1.0; 
        private static final AngularVelocity kMaxFeederSpeed = KrakenX60.kFreeSpeed.div(kFloorReduction);
        public static final AngularVelocity kVelocityTolerance = Units.RPM.of(100);
        public static TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        static {
            // Velocity control (if needed)
            feederConfig.Slot0.kP = 1;
            feederConfig.Slot0.kI = 0;
            feederConfig.Slot0.kD = 0;
            feederConfig.Slot0.kV = 12.0 / kMaxFeederSpeed.in(Units.RotationsPerSecond);

            feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            feederConfig.Feedback.SensorToMechanismRatio = kFloorReduction; 

            feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            feederConfig.CurrentLimits.SupplyCurrentLimit = 70;
            feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            feederConfig.CurrentLimits.StatorCurrentLimit = 120;
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
        }
    }

    
    /**
     * Constants and configuration for the Flip Climber subsystem.
     */
    public static class FlipClimber {
        public static final double kFlipClimberReduction = 80 / 1;
        private static final AngularVelocity kMaxFastClimberSpeed = KrakenX60.kFreeSpeed.div(kFlipClimberReduction);
        
        public static TalonFXConfiguration flipClimberConfig = new TalonFXConfiguration();

        static {
            // position control (if needed)
            flipClimberConfig.Slot0.kP = 1; // definitely too low if we need position control
            flipClimberConfig.Slot0.kI = 0;
            flipClimberConfig.Slot0.kD = 0;
            flipClimberConfig.Slot0.kV = 12.0 / kMaxFastClimberSpeed.in(Units.RotationsPerSecond);

            flipClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            flipClimberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: Find out

            flipClimberConfig.Feedback.SensorToMechanismRatio = kFlipClimberReduction; 

            flipClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            flipClimberConfig.CurrentLimits.SupplyCurrentLimit = 70;
            flipClimberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            flipClimberConfig.CurrentLimits.StatorCurrentLimit = 120;
        }
    }
}
