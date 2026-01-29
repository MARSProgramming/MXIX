package frc.robot.constants;

public class Ports {
    public static class Limelight {
        public static final String kShooterLimelightName = "llshooter";
        public static final String kBackLimelightName = "llback";
    }

    public static class Drivetrain {
        public static final int kFrontRightDrive = 1;
        public static final int kFrontRightSteer = 2;
        
        public static final int kFrontLeftDrive = 7;
        public static final int kFrontLeftSteer = 8;

        public static final int kBackLeftDrive = 5;
        public static final int kBackLeftSteer = 6;

        public static final int kBackRightDrive = 3;
        public static final int kBackRightSteer = 4;
    }

    public static class Flywheel {
        public static final int kLeftFlywheelMaster = 9;
        public static final int kLeftFlywheelFollower = 10;

        public static final int kRightFlywheelMaster = 11;
        public static final int kRightFlywheelFollower = 12;
    }

    public static class Cowl {
        public static final int kCowlMotor = 13;
    }

    public static class Intake {
        public static final int kIntakePivot = 14;
        public static final int kIntakeRollers = 15;
    }

    public static class Floor {
        public static final int kFloorRollers = 16;
    }

    public static class FlipClimber {
        public static final int kTopClimber = 17;
        public static final int kBottomClimber = 18;
        public static final int kLinearActuator = 0; // TODO: Find out
    }

    public static class FastClimber {
        public static final int kHookClimber = 19;
    }

    public static class Feeder {
        public static final int kFeederMotor = 20;
    }
}
