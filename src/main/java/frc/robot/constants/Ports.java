package frc.robot.constants;

/**
 * Constants for the CAN IDs and other ports on the robot.
 * This class organizes constants by subsystem.
 */
public class Ports {
    /**
     * Ports for the Limelight cameras.
     */
    public static class Limelight {
        public static final String kShooterLimelightName = "llshooter";
        public static final String kBackLimelightName = "llback";
    }

    /**
     * CAN IDs for the Swerve Drivetrain modules.
     */
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

    /**
     * CAN IDs for the Flywheel (Shooter) subsystem.
     */
    public static class Flywheel {
        public static final int kLeftFlywheelMaster = 9;
        public static final int kLeftFlywheelFollower = 10;

        public static final int kRightFlywheelMaster = 11;
        public static final int kRightFlywheelFollower = 12;
    }

    /**
     * CAN IDs for the Cowl subsystem.
     */
    public static class Cowl {
        public static final int kCowlMotor = 13;
    }

    /**
     * CAN IDs for the Intake subsystem.
     */
    public static class Intake {
        public static final int kIntakePivot = 14;
        public static final int kIntakeRollers = 15;
    }

    /**
     * CAN IDs for the Floor intake subsystem.
     */
    public static class Floor {
        public static final int kFloorRollers = 16;
    }

    /**
     * CAN IDs for the Flip Climber subsystem.
     */
    public static class FlipClimber {
        public static final int kTopClimber = 17;
        public static final int kBottomClimber = 18;
        public static final int kLinearActuator = 0; // TODO: Find out
    }

    /**
     * CAN IDs for the Fast Climber subsystem.
     */
    public static class FastClimber {
        public static final int kHookClimber = 19;
    }

    /**
     * CAN IDs for the Feeder subsystem.
     */
    public static class Feeder {
        public static final int kFeederMotor = 20;
    }
}
