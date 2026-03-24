package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SystemConstants;
import frc.robot.constants.SystemConstants.Flywheel;
import frc.robot.subsystems.Swerve;

public class ShotSetup {

    private static final double PHASE_DELAY          = 0.05;
    private static final double MAX_COWL_POSITION    = 1.8;
    private static final double RECOMPUTE_THRESHOLD  = 0.05; // meters
    private static final int    CONVERGENCE_ITERS    = 5;
    private static final double CONVERGENCE_EPSILON  = 0.01;
    private static final double MIN_TOF              = 0.05;
    private static final double MAX_TOF              = 1.5;

    // ── Shot map ──────────────────────────────────────────────────────────
    private static final InterpolatingTreeMap<Double, ShotInfo> SHOT_MAP =
        new InterpolatingTreeMap<>(
            (s, e, q) -> InverseInterpolator.forDouble().inverseInterpolate(s, e, q),
            (s, e, t) -> new ShotInfo(
                new Shot(Interpolator.forDouble().interpolate(s.shot.shooterRPM, e.shot.shooterRPM, t)),
                Interpolator.forDouble().interpolate(s.cowlPosition, e.cowlPosition, t)
            )
        );

    // ── Time of flight map ────────────────────────────────────────────────
    private static final InterpolatingTreeMap<Double, Double> TOF_MAP =
        new InterpolatingTreeMap<>(
            (s, e, q) -> InverseInterpolator.forDouble().inverseInterpolate(s, e, q),
            (s, e, t) -> Interpolator.forDouble().interpolate(s, e, t)
        );

    static {
        // ── Shot map entries ──────────────────────────────────────────────
        SHOT_MAP.put(1.24, new ShotInfo(new Shot(3350), 0.50));
        SHOT_MAP.put(2.0,  new ShotInfo(new Shot(3400), 0.70));
        SHOT_MAP.put(2.2,  new ShotInfo(new Shot(3450), 0.80));
        SHOT_MAP.put(2.5,  new ShotInfo(new Shot(3500), 0.95));
        SHOT_MAP.put(3.0,  new ShotInfo(new Shot(3550), 1.10));
        SHOT_MAP.put(3.2,  new ShotInfo(new Shot(3600), 1.15));
        SHOT_MAP.put(3.4,  new ShotInfo(new Shot(3650), 1.20));
        SHOT_MAP.put(3.63, new ShotInfo(new Shot(3700), 1.25));
        SHOT_MAP.put(3.80, new ShotInfo(new Shot(3750), 1.30));
        SHOT_MAP.put(4.0,  new ShotInfo(new Shot(3800), 1.35));
        SHOT_MAP.put(4.2,  new ShotInfo(new Shot(3850), 1.40));
        SHOT_MAP.put(4.4,  new ShotInfo(new Shot(3900), 1.45));
        SHOT_MAP.put(4.6,  new ShotInfo(new Shot(3950), 1.50));
        SHOT_MAP.put(4.8,  new ShotInfo(new Shot(4000), 1.55));
        SHOT_MAP.put(5.0,  new ShotInfo(new Shot(4050), 1.60));
        SHOT_MAP.put(5.2,  new ShotInfo(new Shot(4100), 1.65));
        SHOT_MAP.put(5.4,  new ShotInfo(new Shot(4150), 1.70));
        SHOT_MAP.put(5.6,  new ShotInfo(new Shot(4200), 1.75));

        // ── Time of flight entries ─────────────────────────────────────────
        TOF_MAP.put(1.24, 1.310);
        TOF_MAP.put(2.0,  1.330);
        TOF_MAP.put(3.0,  1.335);
        TOF_MAP.put(4.0,  1.340);
        TOF_MAP.put(5.6,  1.350);
    }

    // ── Cache — one per target type ───────────────────────────────────────
    private SOTMInfo mCachedHubInfo     = null;
    private double   mLastHubDist       = -1;

    private SOTMInfo mCachedShuttleInfo = null;
    private double   mLastShuttleDist   = -1;

    // ─────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Static shot — no SOTM compensation. Use when the robot is stationary.
     */
    public ShotInfo getStaticShotInfo(double hubDistMeters) {
        ShotInfo raw = SHOT_MAP.get(hubDistMeters);
        return clamp(raw);
    }

    /**
     * SOTM shot info for the hub. Cached — only recomputes when distance
     * changes by more than RECOMPUTE_THRESHOLD.
     */
    public SOTMInfo getSOTMInfoHub(Swerve swerve) {
        Translation2d target = FieldConstants.Locations.hubPosition();
        double currentDist   = target.getDistance(swerve.getState().Pose.getTranslation());

        if (mCachedHubInfo != null
                && Math.abs(currentDist - mLastHubDist) < RECOMPUTE_THRESHOLD) {
            return mCachedHubInfo;
        }

        mLastHubDist    = currentDist;
        mCachedHubInfo  = computeSOTM(swerve, target, true);
        return mCachedHubInfo;
    }

    /**
     * SOTM shot info for the shuttle target. Cached — only recomputes when
     * distance changes by more than RECOMPUTE_THRESHOLD.
     */
    public SOTMInfo getSOTMInfoShuttle(Swerve swerve) {
        Translation2d target = FieldConstants.Locations
            .getClosestShuttlingPosition(swerve.getState().Pose).getTranslation();
        double currentDist   = target.getDistance(swerve.getState().Pose.getTranslation());

        if (mCachedShuttleInfo != null
                && Math.abs(currentDist - mLastShuttleDist) < RECOMPUTE_THRESHOLD) {
            return mCachedShuttleInfo;
        }

        mLastShuttleDist    = currentDist;
        mCachedShuttleInfo  = computeSOTM(swerve, target, false);
        return mCachedShuttleInfo;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Core SOTM computation — shared by hub and shuttle
    // ─────────────────────────────────────────────────────────────────────

    private SOTMInfo computeSOTM(Swerve swerve, Translation2d target, boolean isHub) {
        var state = swerve.getState();

        // ── Phase-delay compensated pose ──────────────────────────────────
        Pose2d robotPose = state.Pose.exp(new Twist2d(
            state.Speeds.vxMetersPerSecond * PHASE_DELAY,
            state.Speeds.vyMetersPerSecond * PHASE_DELAY,
            state.Speeds.omegaRadiansPerSecond * PHASE_DELAY
        ));

        Pose2d shotPose = robotPose.plus(
            GeometryUtil.toTransform2d(SystemConstants.Flywheel.ROBOT_TO_SHOOTER_TRANSFORM));

        // ── Field-relative shooter velocity (robot + rotational component) ─
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            state.Speeds, robotPose.getRotation());

        var shooterTransform = SystemConstants.Flywheel.ROBOT_TO_SHOOTER_TRANSFORM;
        double cosH = robotPose.getRotation().getCos();
        double sinH = robotPose.getRotation().getSin();
        double offsetX =  shooterTransform.getX() * cosH - shooterTransform.getY() * sinH;
        double offsetY =  shooterTransform.getX() * sinH + shooterTransform.getY() * cosH;
        double omega   = fieldSpeeds.omegaRadiansPerSecond;

        double velX = fieldSpeeds.vxMetersPerSecond + (-offsetY) * omega;
        double velY = fieldSpeeds.vyMetersPerSecond + ( offsetX) * omega;

        // ── Iterative convergence ─────────────────────────────────────────
        Pose2d  lookaheadPose = shotPose;
        double  lookaheadDist = target.getDistance(shotPose.getTranslation());

        for (int i = 0; i < CONVERGENCE_ITERS; i++) {
            double tof = MathUtil.clamp(TOF_MAP.get(lookaheadDist), MIN_TOF, MAX_TOF);

            lookaheadPose = new Pose2d(
                shotPose.getTranslation().plus(
                    new Translation2d(velX * tof, velY * tof)),
                shotPose.getRotation()
            );

            double newDist = target.getDistance(lookaheadPose.getTranslation());
            if (Math.abs(newDist - lookaheadDist) < CONVERGENCE_EPSILON) break;
            lookaheadDist = newDist;
        }

        // ── Shot params ───────────────────────────────────────────────────
        ShotInfo clamped = clamp(SHOT_MAP.get(lookaheadDist));

        // ── Angular velocity feedforward ──────────────────────────────────
        double angularVelocity = 0;
        if (lookaheadDist > 0.1) {
            double rx = target.getX() - lookaheadPose.getX();
            double ry = target.getY() - lookaheadPose.getY();
            angularVelocity = (ry * velX - rx * velY) / (lookaheadDist * lookaheadDist);
        }

        // ── Lookahead direction ───────────────────────────────────────────
        Rotation2d direction = getLookaheadDirection(lookaheadPose, swerve, isHub);

        return new SOTMInfo(clamped, direction, angularVelocity);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────

    private ShotInfo clamp(ShotInfo raw) {
        return new ShotInfo(
            new Shot(MathUtil.clamp(raw.shot.shooterRPM, 0,
                Flywheel.kMaxFlywheelSpeed.in(Units.RPM))),
            MathUtil.clamp(raw.cowlPosition, 0, MAX_COWL_POSITION)
        );
    }

    private Rotation2d getLookaheadDirection(Pose2d lookaheadPose, Swerve swerve, boolean isHub) {
        Translation2d target = isHub
            ? FieldConstants.Locations.hubPosition()
            : FieldConstants.Locations.getClosestShuttlingPosition(lookaheadPose).getTranslation();

        Rotation2d directionBlue = target.minus(lookaheadPose.getTranslation()).getAngle();
        return directionBlue.rotateBy(swerve.getOperatorForwardDirection());
    }

    // ─────────────────────────────────────────────────────────────────────
    // Data classes
    // ─────────────────────────────────────────────────────────────────────

    public static class Shot {
        public final double shooterRPM;
        public Shot(double shooterRPM) { this.shooterRPM = shooterRPM; }
    }

    public static class ShotInfo {
        public final Shot   shot;
        public final double cowlPosition;
        public ShotInfo(Shot shot, double cowlPosition) {
            this.shot         = shot;
            this.cowlPosition = cowlPosition;
        }
    }

    public static class SOTMInfo {
        public final ShotInfo  shotInfo;
        public final Rotation2d virtualTargetAngle;
        public final double     angularVelocityRadPerSec;
        public SOTMInfo(ShotInfo shotInfo, Rotation2d virtualTargetAngle, double angularVelocityRadPerSec) {
            this.shotInfo                = shotInfo;
            this.virtualTargetAngle      = virtualTargetAngle;
            this.angularVelocityRadPerSec = angularVelocityRadPerSec;
        }
    }
}