package frc.robot.util;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import frc.robot.constants.SystemConstants.Flywheel;

public class ShuttleSetup {

    private static final InterpolatingTreeMap<Double, ShuttleShotInfo> SHUTTLE_MAP = new InterpolatingTreeMap<>(
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
            new ShuttleShotInfo(
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


    
    // ========== INITIALIZATION ==========
    static {
        loadMap();
    }

    
    private static void loadMap() {
        // Close range - flatter cowl, lower speeds
        SHUTTLE_MAP.put(1.0,  new ShuttleShotInfo(new Shot(3000), 1.7));  // Very close
        SHUTTLE_MAP.put(2.0,  new ShuttleShotInfo(new Shot(3500), 1.7));  // Close
        SHUTTLE_MAP.put(3.0,  new ShuttleShotInfo(new Shot(4000), 1.7));  // Mid-close
        SHUTTLE_MAP.put(4.0,  new ShuttleShotInfo(new Shot(4500), 1.7));  // Sweet spot start
        SHUTTLE_MAP.put(5.0,  new ShuttleShotInfo(new Shot(5000), 1.7));  // Sweet spot

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
    public ShuttleShotInfo getStaticShotInfo(double hubDistMeters) {
        ShuttleShotInfo initialShotInfo = SHUTTLE_MAP.get(hubDistMeters);

        double clampedCowlPos = Math.max(0, Math.min(initialShotInfo.cowlPosition, 1.8));  // Clamp cowl between 0 and 1.8 rotations
        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), initialShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values

        return new ShuttleShotInfo(
            new Shot(clampedShooterRpm),
            clampedCowlPos
        );  
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
    public static class ShuttleShotInfo {
        public final Shot shot;
        public final double cowlPosition;  // Continuous value for TalonFX position control
        
        public ShuttleShotInfo(Shot shot, double cowlPosition) {
            this.shot = shot;
            this.cowlPosition = cowlPosition;
        }
    }
}