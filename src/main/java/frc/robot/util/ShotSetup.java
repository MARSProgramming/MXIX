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

    

    private static final InterpolatingTreeMap<Distance, ShotInfo> SHOT_MAP = new InterpolatingTreeMap<>(
        // Inverse interpolator: "where does this query distance sit between two calibration points?"
        (startDistance, endDistance, queryDistance) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(
                    startDistance.in(Meters), 
                    endDistance.in(Meters), 
                    queryDistance.in(Meters)
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
        SHOT_MAP.put(Meters.of(1.5),  new ShotInfo(new Shot(2500), 0.0));   // Point blank
        SHOT_MAP.put(Meters.of(2.0),  new ShotInfo(new Shot(2600), 0.05));  // Very close
        SHOT_MAP.put(Meters.of(2.5),  new ShotInfo(new Shot(2700), 0.15));  // Close
        
        // Transition range - common shooting zone, add extra points
        SHOT_MAP.put(Meters.of(3.0),  new ShotInfo(new Shot(2850), 0.30));  // Mid-close
        SHOT_MAP.put(Meters.of(3.5),  new ShotInfo(new Shot(2950), 0.40));  // Sweet spot start
        SHOT_MAP.put(Meters.of(4.0),  new ShotInfo(new Shot(3050), 0.50));  // Sweet spot
        SHOT_MAP.put(Meters.of(4.5),  new ShotInfo(new Shot(3150), 0.60));  // Sweet spot
        SHOT_MAP.put(Meters.of(5.0),  new ShotInfo(new Shot(3250), 0.70));  // Sweet spot end
        
        // Mid-long range
        SHOT_MAP.put(Meters.of(5.5),  new ShotInfo(new Shot(3350), 0.80));  // Getting far
        SHOT_MAP.put(Meters.of(6.0),  new ShotInfo(new Shot(3450), 0.90));  // Far
        SHOT_MAP.put(Meters.of(6.5),  new ShotInfo(new Shot(3600), 1.00));  // Very far
        
        // Long range - steeper cowl, higher speeds
        SHOT_MAP.put(Meters.of(7.0),  new ShotInfo(new Shot(3750), 1.10));  // Long
        SHOT_MAP.put(Meters.of(8.0),  new ShotInfo(new Shot(3950), 1.25));  // Very long
        SHOT_MAP.put(Meters.of(9.0),  new ShotInfo(new Shot(4100), 1.40));  // Max practical
        SHOT_MAP.put(Meters.of(10.0), new ShotInfo(new Shot(4250), 1.50));  // Absolute max

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);


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
    public ShotInfo getStaticShotInfo(Distance distanceToHub) {
        ShotInfo initialShotInfo = SHOT_MAP.get(distanceToHub);

        double clampedCowlPos = Math.max(0, Math.min(initialShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations
        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), initialShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values

        return new ShotInfo(
            new Shot(clampedShooterRpm),
            clampedCowlPos
        );  
    }

    public SOTMInfo getSOTMInfo(Swerve swerveSubsystem) {
        double phaseDelay = 0.03;
        Pose2d estimatedShotPose = swerveSubsystem.getState().Pose;
        estimatedShotPose = 
            estimatedShotPose.exp(new 
            Twist2d(swerveSubsystem.getState().Speeds.vxMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.vyMetersPerSecond * phaseDelay,
            swerveSubsystem.getState().Speeds.omegaRadiansPerSecond * phaseDelay));

        Translation2d target = FieldConstants.Locations.hubPosition(); // add alliance util logic to get this target (hub target)
        Distance launcherToTargetDistance = Units.Meters.of(target.getDistance(estimatedShotPose.getTranslation()));

        double relativeVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vxMetersPerSecond;
        double relativeVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getState().Speeds, swerveSubsystem.getState().Pose.getRotation()).vyMetersPerSecond;

        double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance.magnitude());

        Pose2d lookaheadPose = estimatedShotPose;
        Distance lookaheadLauncherToTargetDist = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDist.magnitude());
            double offsetX = relativeVelocityX * timeOfFlight;
            double offsetY = relativeVelocityY * timeOfFlight;

            lookaheadPose = new Pose2d(
                estimatedShotPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                estimatedShotPose.getRotation()
            );

            lookaheadLauncherToTargetDist = Units.Meters.of(target.getDistance(lookaheadPose.getTranslation()));
        }


        ShotInfo desiredShotInfo = SHOT_MAP.get(lookaheadLauncherToTargetDist);

        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), desiredShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values
        double clampedCowlPos = Math.max(0, Math.min(desiredShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations

        ShotInfo clampedDesiredShotInfo = new ShotInfo(new Shot(clampedShooterRpm), clampedCowlPos);

        return new SOTMInfo(clampedDesiredShotInfo, getDirectionToHub(swerveSubsystem));
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

}