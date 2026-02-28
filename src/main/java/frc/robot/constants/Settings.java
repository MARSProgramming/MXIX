package frc.robot.constants;

public class Settings {
    
    public static class IntakePivotSettings {
        public static double INTAKE_DEPLOY_TIMEOUT = 0.45;
        public static double INTAKE_DEPLOYMENT_DUTYCYCLE = 0.5;
    }

    public static class FeedSystemSettings {
        public static double FEEDER_FEED_DUTYCYCLE = 1.0;
        public static double FLOOR_FEED_DUTYCYCLE = 0.8;
        public static double INTAKEROLLER_FEED_DUTYCYCLE = 0.8;

        public static double UNJAM_DUTYCYCLE = -0.5;
    }

    public static class IntakeSystemSettings {
        public static double INTAKING_STANDARD_DUTYCYCLE = 0.8;
        public static double INTAKING_FLOOR_STANDARD_DUTYCYCLE = 0.2;
    }
}
