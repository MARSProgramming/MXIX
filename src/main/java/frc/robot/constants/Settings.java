package frc.robot.constants;

public class Settings {
    
    public static class IntakePivotSettings {
        public static double INTAKE_DEPLOY_TIMEOUT = 0.3;
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
        public static double LINEUP_VELOCITY = 0.3; // M/s
        public static double SIDEWAYS_LINEUP_TIMEOUT = 1.0;
        public static double FORWARD_LINEUP_TIMEOUT = 0.7;
    }

    public static class ClimbSettings {
        public static double CLIMB_DUTYCYCLE = 0.4;
    }

    public static class ReferenceShotSettings {
        public static double HUB_REFERENCE_FLYWHEEL_VELOCITY = 3400;
        public static double HUB_REFERENCE_COWL_POSITION = 0.4;
    }
}