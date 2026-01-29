package frc.robot.constants;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class SystemConstants {
    
    public static class AutoAim {

        // SOTM
        public static final double AUTO_AIM_MAX_DRIVETRAIN_VEL = 6.4; 
        // Compensation fudge factor for rotation
        public static final double ROTATION_VELOCITY_COMPENSATION_FACTOR = 0.1;

        //Turn to pose constants
        public static final double AUTO_AIM_MARGIN = 1.5;
        public static final double AUTO_AIM_SETPOINT_MARGIN = 5; // Degrees
        public static final double AUTO_AIM_MAX_VEL_SETPOINT = 1.5; // The maximum commanded rotational velocity in rad/s
        public static final double AUTO_AIM_MAX_ROT_VEL = 4.4;
        public static final double AUTO_AIM_KP = 0.1;
        public static final double AUTO_AIM_KD = 0; 

    }


    public static class Drive {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = Units.RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = Units.RPM.of(6000);
    }
}
