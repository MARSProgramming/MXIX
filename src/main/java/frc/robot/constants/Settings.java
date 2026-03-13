package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Settings {
    
    public static class IntakePivotSettings {
        public static double INTAKE_DEPLOY_TIMEOUT = 0.2;
        public static double INTAKE_RETRACT_TIMEOUT = 0.3;
        public static double INTAKE_DEPLOYMENT_DUTYCYCLE = 0.6;
        public static double INTAKE_LIFT_DUTYCYCLE = 0.7;

    }

    public static class FeedSystemSettings {
        public static double FEEDER_FEED_DUTYCYCLE = 1.0;
        public static double FLOOR_FEED_DUTYCYCLE = 0.8;
        public static double INTAKEROLLER_FEED_DUTYCYCLE = 0.8;

        public static double UNJAM_DUTYCYCLE = -0.5;
    }

    public static class IntakeSystemSettings {
        public static double INTAKING_STANDARD_DUTYCYCLE = 0.9;
        public static double INTAKING_FLOOR_DUTYCYCLE = 0.05;
        public static double INTAKING_FEEDER_DUTYCYCLE = -0.05;
    }


    public static class ClimbLineupSettings {
        public static double SIDELINEUP_VELOCITY = 0.3; // M/s
        public static double FORWLINEUP_VELOCITY = 0.2; // M/s

        // probably slwo this down, particularly the forwards one, the forwards one can't
        // dealign the climber.
        public static double SIDEWAYS_LINEUP_TIMEOUT = 2.0;
        public static double FORWARD_LINEUP_TIMEOUT = 1.5;
    }

    public static class ClimbSettings {
        public static double CLIMB_DUTYCYCLE = 0.4;
    }

    public static class ReferenceShotSettings {
        public static double HUB_REFERENCE_FLYWHEEL_VELOCITY = 3400;
        public static double HUB_REFERENCE_COWL_POSITION = 0.4;
    }

    public static class ShooterSettings {
        public static double FLYWHEEL_MIN_RPMs = 3000;
    }

      public static final class kAutoAlign {
    public static final PIDConstants ALIGN_PID = new PIDConstants(3.5, 0.0, 0);

    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY_SLOW     = Units.MetersPerSecond.of(2.00);
    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY_FAST     = Units.MetersPerSecond.of(1.5);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION_SLOW = Units.MetersPerSecondPerSecond.of(8.00);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION_FAST = Units.MetersPerSecondPerSecond.of(4.0);

    /* ------------ WHO'S IN THE HOUSE?? ------------- */
    public static final Distance TRANSLATION_TOLERANCE;
    public static final Angle    ROTATION_TOLERANCE   ;
    public static final LinearVelocity VELOCITY_TOLERANCE = Units.MetersPerSecond.of(0.18);
    public static final LinearVelocity AUTO_VELOCITY_TOLERANCE = Units.MetersPerSecond.of(0.15);
    static {
            TRANSLATION_TOLERANCE = Units.Centimeters.of(3.0);
            ROTATION_TOLERANCE    = Units.Degrees.of(2);
        }
    }    

    
}