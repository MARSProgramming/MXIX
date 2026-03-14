package frc.robot.util;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.constants.SystemConstants.Flywheel;
import frc.robot.subsystems.Swerve;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Locations;

public class ShotSetup {

    

    private static final InterpolatingTreeMap<Double, ShotInfo> SHOT_MAP = new InterpolatingTreeMap<>(
        // Inverse interpolator: "where does this query distance sit between two calibration points?"
        (startDistance, endDistance, queryDistance) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(
                    startDistance, 
                    endDistance, 
                    queryDistance
                ),
        
        // Forward interpolator: "given position 't' between two ShotInfos, interpolate both fields"
        (startInfo, endInfo, t) -> 
            new ShotInfo(
                new Shot(
                    // Interpolate RPM
                    Interpolator.forDouble()
                        .interpolate(startInfo.shot.shooterRPM, endInfo.shot.shooterRPM, t)
                ),
                // Interpolate cowl position (this is the key - cowl is now continuous!)
                Interpolator.forDouble()
                    .interpolate(startInfo.cowlPosition, endInfo.cowlPosition, t)
            )
    );

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

    
    // ========== INITIALIZATION ==========
    static {
        loadMap();
    }

    
    private static void loadMap() {
        // Close range - flatter cowl, lower speeds
        SHOT_MAP.put(1.24,  new ShotInfo(new Shot(3350), 0.5));   // tuned
        SHOT_MAP.put(2.0,  new ShotInfo(new Shot(3450), 0.7));  // tuned
        SHOT_MAP.put(2.2,  new ShotInfo(new Shot(3500), 0.8));  // needs tuning
        SHOT_MAP.put(2.5,  new ShotInfo(new Shot(3550), 0.95));  // needs tuning
        SHOT_MAP.put(3.0,  new ShotInfo(new Shot(3650), 1.1));  // needs tuning
        SHOT_MAP.put(3.2,  new ShotInfo(new Shot(3700), 1.15));  // needs tuning
        SHOT_MAP.put(3.4,  new ShotInfo(new Shot(3750), 1.2));  // needs tuning
        SHOT_MAP.put(3.63,  new ShotInfo(new Shot(3800), 1.25));  // needs tuning
        SHOT_MAP.put(3.80,  new ShotInfo(new Shot(3850), 1.3));  // needs tuning
        SHOT_MAP.put(4.0,  new ShotInfo(new Shot(3900), 1.35));  // needs tuning
        SHOT_MAP.put(4.2,  new ShotInfo(new Shot(3950), 1.4));  // needs tuning
        SHOT_MAP.put(4.4,  new ShotInfo(new Shot(4000), 1.45));  // needs tuning
        SHOT_MAP.put(4.6,  new ShotInfo(new Shot(4050), 1.5));  // needs tuning
        SHOT_MAP.put(4.8,  new ShotInfo(new Shot(4100), 1.55));  // needs tuning
        SHOT_MAP.put(5.0,  new ShotInfo(new Shot(4150), 1.6));  // needs tuning
        SHOT_MAP.put(5.2,  new ShotInfo(new Shot(4200), 1.65));  // needs tuning
        SHOT_MAP.put(5.4,  new ShotInfo(new Shot(4250), 1.70));  // needs tuning
        SHOT_MAP.put(5.6,  new ShotInfo(new Shot(4300), 1.75));  // needs tuning


        timeOfFlightMap.put(5.68, 1.4);
        timeOfFlightMap.put(4.55, 1.35);
        timeOfFlightMap.put(3.15, 1.3);
        timeOfFlightMap.put(1.88, 1.25);
        timeOfFlightMap.put(1.24, 1.2);


    }
    
    
    // ========== PUBLIC API ==========
    
    /**
     * Determines optimal shot parameters for a given distance to target.
     * 
     * Returns interpolated RPM and cowl position with NO discrete jumps.
     * Both values smoothly transition across the entire distance range.
     * 
     * CLAMPING BEHAVIOR:
     * - Distances below minimum: Uses minimum calibration data (doesn't extrapolate down)
     * - Distances above maximum: THROWS EXCEPTION (don't attempt uncalibrated long shots)
     * 
     * @param distanceToHub Distance from robot to scoring target
     * @return ShotInfo with continuously interpolated RPM and cowl position
     * @throws IllegalArgumentException if distance is negative or beyond calibrated maximum
     */
    public ShotInfo getStaticShotInfo(double hubDistMeters) {
        ShotInfo initialShotInfo = SHOT_MAP.get(hubDistMeters);

        double clampedCowlPos = Math.max(0, Math.min(initialShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations
        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), initialShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values

        return new ShotInfo(
            new Shot(clampedShooterRpm),
            clampedCowlPos
        );  
    }

    public SOTMInfo getSOTMInfoHub(Swerve swerveSubsystem) {
        double phaseDelay = 0.03;
        Pose2d estimatedShotPose = swerveSubsystem.getState().Pose;
        estimatedShotPose = 
            estimatedShotPose.exp(new 
            Twist2d(swerveSubsystem.getState().Speeds.vxMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.vyMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.omegaRadiansPerSecond * phaseDelay));

        Translation2d target = FieldConstants.Locations.hubPosition(); // add alliance util logic to get this target (hub target)
        double launcherToTargetDistance = target.getDistance(estimatedShotPose.getTranslation());

        double relativeVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vxMetersPerSecond;
        double relativeVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vyMetersPerSecond;

        double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);

        Pose2d lookaheadPose = estimatedShotPose;
        double lookaheadLauncherToTargetDist = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDist);
            double offsetX = relativeVelocityX * timeOfFlight;
            double offsetY = relativeVelocityY * timeOfFlight;

            lookaheadPose = new Pose2d(
                estimatedShotPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                estimatedShotPose.getRotation()
            );

            lookaheadLauncherToTargetDist = target.getDistance(lookaheadPose.getTranslation());
        }


        ShotInfo desiredShotInfo = SHOT_MAP.get(lookaheadLauncherToTargetDist);

        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), desiredShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values
        double clampedCowlPos = Math.max(0, Math.min(desiredShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations

        ShotInfo clampedDesiredShotInfo = new ShotInfo(new Shot(clampedShooterRpm), clampedCowlPos);

        return new SOTMInfo(clampedDesiredShotInfo, getLookaheadDirection(lookaheadPose, swerveSubsystem, true));
    }

    public SOTMInfo getSOTMInfoShuttle(Swerve swerveSubsystem) {
        double phaseDelay = 0.03;
        Pose2d estimatedShotPose = swerveSubsystem.getState().Pose;
        estimatedShotPose = 
            estimatedShotPose.exp(new 
            Twist2d(swerveSubsystem.getState().Speeds.vxMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.vyMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.omegaRadiansPerSecond * phaseDelay));

        Translation2d target = FieldConstants.Locations.getClosestShuttlingPosition(estimatedShotPose).getTranslation(); // add alliance util logic to get this target (hub target)
        double launcherToTargetDistance = target.getDistance(estimatedShotPose.getTranslation());

        double relativeVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vxMetersPerSecond;
        double relativeVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vyMetersPerSecond;

        double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);

        Pose2d lookaheadPose = estimatedShotPose;
        double lookaheadLauncherToTargetDist = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDist);
            double offsetX = relativeVelocityX * timeOfFlight;
            double offsetY = relativeVelocityY * timeOfFlight;

            lookaheadPose = new Pose2d(
                estimatedShotPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                estimatedShotPose.getRotation()
            );

            lookaheadLauncherToTargetDist = target.getDistance(lookaheadPose.getTranslation());
        }


        ShotInfo desiredShotInfo = SHOT_MAP.get(lookaheadLauncherToTargetDist);

        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), desiredShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values
        double clampedCowlPos = Math.max(0, Math.min(desiredShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations

        ShotInfo clampedDesiredShotInfo = new ShotInfo(new Shot(clampedShooterRpm), clampedCowlPos);

        return new SOTMInfo(clampedDesiredShotInfo, getLookaheadDirection(lookaheadPose, swerveSubsystem, false));
    }


    // ========== DATA CLASSES ==========
    
    /**
     * Represents flywheel parameters for a shot.
     * Currently only RPM, but could be extended for:
     * - Dual-wheel differential spin (for shot shaping)
     * - Feeder speed coordination
     * - Spin rate for stability
     */
    public static class Shot {
        public final double shooterRPM;
        
        public Shot(double shooterRPM) {
            this.shooterRPM = shooterRPM;
        }
    }
    
    /**
     * Complete shot configuration: flywheel RPM + cowl position.
     * Both values are now continuously interpolated - no discrete jumps.
     * 
     * This is what gets sent to your shooter subsystem.
     */
    public static class ShotInfo {
        public final Shot shot;
        public final double cowlPosition;  // Continuous value for TalonFX position control
        
        public ShotInfo(Shot shot, double cowlPosition) {
            this.shot = shot;
            this.cowlPosition = cowlPosition;
        }
    }

    public static class SOTMInfo {
        public ShotInfo shotInfo;
        public Rotation2d virtualTargetAngle;

        public SOTMInfo(ShotInfo shotInfo, Rotation2d virtualTargetAngle) {
            this.shotInfo = shotInfo;
            this.virtualTargetAngle = virtualTargetAngle;
        }   
    }

    private Rotation2d getDirectionToHub(Swerve swerveSubsystem) {
        final Translation2d hubPosition = Locations.hubPosition();
        final Translation2d robotPosition = swerveSubsystem.getState().Pose.getTranslation();
        
        // Calculate angle in standard field coordinates (Blue Alliance origin)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        
        // Adjust for the driver's perspective (Red vs Blue alliance)
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerveSubsystem.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    private Rotation2d getLookaheadDirection(Pose2d lookaheadPose, Swerve swerveSubsystem, boolean atHub) {
        final Translation2d hubPosition = Locations.hubPosition();
        final Translation2d desiredShuttlePos = Locations.getClosestShuttlingPosition(lookaheadPose).getTranslation();
        final Translation2d lookaheadPosition = lookaheadPose.getTranslation();
        
        // Calculate angle in standard field coordinates (Blue Alliance origin)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(lookaheadPosition).getAngle();
        final Rotation2d shuttleDirectionInBlueAlliancePerspective = desiredShuttlePos.minus(lookaheadPosition).getAngle();

        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerveSubsystem.getOperatorForwardDirection());
        final Rotation2d shuttleDirectionInOperatorPerspective = shuttleDirectionInBlueAlliancePerspective.rotateBy(swerveSubsystem.getOperatorForwardDirection());
        if (atHub) {
            return hubDirectionInOperatorPerspective;
        } else {
            return shuttleDirectionInOperatorPerspective;
        }
    }
}