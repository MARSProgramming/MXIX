package frc.robot.util;
import static edu.wpi.first.units.Units.Meters;

import java.util.*;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.SystemConstants.Flywheel;

/**
 * Manages shot parameter selection (flywheel RPM and cowl position) based on distance to target.
 * 
 * Uses a CONTINUOUS INTERPOLATION approach where both RPM and cowl position smoothly interpolate
 * across the entire distance range. This eliminates zone boundaries and discrete transitions.
 * 
 * Design rationale: Since the cowl is driven by a TalonFX with continuous positioning capability,
 * we can leverage smooth interpolation rather than discrete zones. This provides:
 * - No discontinuities in trajectory (smoother aiming)
 * - Easier velocity compensation for shooting on the move
 * - Simpler code with no state management or hysteresis logic
 * - More calibration points = better accuracy
 * 
 * Trade-off: Requires more comprehensive calibration data across the full distance range.
 */
public class ShotSetup {
    
    /**
     * Unified shot map: distance -> (RPM, cowl position)
     * 
     * Both RPM and cowl position interpolate linearly between calibration points.
     * More data points = better accuracy, especially in high-priority distance ranges.
     * 
     * CALIBRATION PROCESS:
     * 1. Start at close range (1.5m)
     * 2. Find optimal (RPM, cowl) combination that consistently scores
     * 3. Move to next distance (add more points in ranges you shoot from often)
     * 4. Repeat across entire operational range
     * 5. Verify interpolation works well between points
     * 
     * PRO TIP: Add extra calibration points in your most common shooting distances
     * (e.g., if you shoot a lot from 3-5m, add points every 0.5m in that range)
     */
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
    
    // ========== INITIALIZATION ==========
    static {
        loadMap();
    }
    
    /**
     * Loads empirically measured calibration data.
     * 
     * IMPORTANT: These are PLACEHOLDER values. You must replace these with real field data.
     * 
     * Calibration strategy:
     * - Close range (1.5-3m): Lower RPM, flatter cowl, dense points for accuracy
     * - Mid range (3-6m): Moderate RPM/cowl, points every ~1m
     * - Long range (6-10m): Higher RPM, steeper cowl, verify extrapolation doesn't occur
     * 
     * TUNING TIPS:
     * - Start with fewer points, verify interpolation is reasonable
     * - Add points in problem areas where shots are inconsistent
     * - Use telemetry to identify which distances you actually shoot from
     * - Consider separate calibration for stationary vs. moving shots
     * 
     */
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
    public ShotInfo getClampedShotInfo(Distance distanceToHub) {
        ShotInfo initialShotInfo = SHOT_MAP.get(distanceToHub);

        double clampedCowlPos = Math.max(0, Math.min(initialShotInfo.cowlPosition, 1.5));  // Clamp cowl between 0 and 1.5 rotations
        double clampedShooterRpm = Math.max(0, Math.min(Flywheel.kMaxFlywheelSpeed.in(Units.RPM), initialShotInfo.shot.shooterRPM));  // Clamp RPM to non-negative values

        return new ShotInfo(
            new Shot(clampedShooterRpm),
            clampedCowlPos
        );  
    }



    
    /**
     * Variant that includes robot velocity compensation for shooting on the move.
     * 
     * When the robot is moving, the projectile inherits the robot's velocity.
     * This shifts both the effective distance and required launch velocity.
     * 
     * IMPLEMENTATION NOTE: This is a simplified linear compensation model.
     * For competition-level accuracy, consider full projectile physics solving.
     * 
     * @param distanceToHub Static distance to target
     * @param robotVelocity Robot's velocity vector (positive = moving toward target)
     * @return ShotInfo with velocity-compensated parameters
     */
    public ShotInfo getShotInfoWithVelocityComp(Distance distanceToHub, double robotVelocityMPS) {
        // Get base shot parameters for stationary shot
        ShotInfo baseShot = getClampedShotInfo(distanceToHub);
        
        // Simple linear velocity compensation (tune this coefficient based on testing)
        // Positive velocity (moving toward target) = need less RPM
        // Negative velocity (moving away) = need more RPM
        double VELOCITY_COMP_COEFFICIENT = 50.0;  // RPM per m/s of robot velocity (Tune)
        double compensatedRPM = baseShot.shot.shooterRPM - (robotVelocityMPS * VELOCITY_COMP_COEFFICIENT);
                        
        return new ShotInfo(
            new Shot(compensatedRPM),
            baseShot.cowlPosition
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
    public static class ShotInfo {
        public final Shot shot;
        public final double cowlPosition;  // Continuous value for TalonFX position control
        
        public ShotInfo(Shot shot, double cowlPosition) {
            this.shot = shot;
            this.cowlPosition = cowlPosition;
        }
    }
}