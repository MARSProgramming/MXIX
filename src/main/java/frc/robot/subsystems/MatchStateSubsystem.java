package frc.robot.subsystems;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tracks match state for FRC Rebuilt (2026).
 *
 * Uses an elapsed timer started at teleop init for reliability —
 * DriverStation.getMatchTime() returns -1 without FMS.
 *
 * Hub shift timing (elapsed seconds from teleop start):
 *   TRANSITION SHIFT: 0:00 – 0:10  (both hubs active)
 *   SHIFT 1:          0:10 – 0:35
 *   SHIFT 2:          0:35 – 1:00
 *   SHIFT 3:          1:00 – 1:25
 *   SHIFT 4:          1:25 – 1:50
 *   END GAME:         1:50 – 2:20  (both hubs active)
 *
 * If we won auto, SHIFT 4 + END GAME are merged into one continuous
 * 55-second active window since our hub is active for both periods.
 *
 * Auto winner's hub is INACTIVE in SHIFT 1.
 *
 * ===============================================================
 * SETUP: call mMatchStateSystem.onTeleopInit() from Robot.teleopInit()
 * ===============================================================
 *
 * TESTING WITHOUT FMS (Practice Matches):
 *   1. Open FRC Driver Station → Setup tab → Game Data field
 *   2. Enter BEFORE or AFTER enabling teleop:
 *        "R" → Red won auto  (Red hub inactive in Shift 1)
 *        "B" → Blue won auto (Blue hub inactive in Shift 1)
 *   3. Leave empty → isHubActive() always returns true,
 *        wonAuto() always false (no hub alternation)
 *   4. Game data locks in on the first periodic cycle it is seen
 *
 * Verify in DogLog:
 *   - MatchState/WonAuto                  → locks in at teleop start
 *   - MatchState/HubActive               → flips correctly each shift
 *   - MatchState/CurrentShift            → transitions at correct times
 *   - MatchState/SecondsRemainingInShift
 *   - MatchState/ShiftWarning            → true for one cycle at 5s left
 *   - MatchState/ShiftStart              → true for one cycle at shift start
 * ================================================================
 */
public class MatchStateSubsystem extends SubsystemBase {

    // ── Shift boundaries (elapsed seconds from teleop start) ──────────────
    private static final double TRANSITION_END        = 10.0;
    private static final double SHIFT_1_END           = 35.0;
    private static final double SHIFT_2_END           = 60.0;
    private static final double SHIFT_3_END           = 85.0;
    private static final double SHIFT_4_END           = 110.0;
    private static final double MATCH_END             = 140.0;
    private static final double SHIFT_WARNING_THRESHOLD = 5.0;

    // ── Timer ──────────────────────────────────────────────────────────────
    private final Timer mTeleopTimer = new Timer();

    // ── Auto result ────────────────────────────────────────────────────────
    private boolean mWonAuto          = false;
    private boolean mAutoResultLocked = false;
    private boolean mOurHubActiveShift1 = true;

    // ── Shift change tracking (computed once per periodic) ─────────────────
    private Shift   mLastShift          = Shift.DISABLED;
    private boolean mShiftChanged       = false;

    // ── Rumble flags ───────────────────────────────────────────────────────
    private boolean mShiftWarningFired  = false;
    private boolean mShiftStartFired    = false;

    private boolean mShouldRumbleWarning = false;
    private boolean mShouldRumbleStart = false;

    public enum Shift {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        DISABLED
    }

    public MatchStateSubsystem() {}

    // ── Teleop init ────────────────────────────────────────────────────────

    /** Must be called from Robot.teleopInit() */
    public void onTeleopInit() {
        mTeleopTimer.restart();
    }

    // ── Auto result ────────────────────────────────────────────────────────

    private void tryLockAutoResult() {
        if (mAutoResultLocked) return;
        if (!DriverStation.isTeleopEnabled()) return;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return;

        boolean redWonAuto = gameData.charAt(0) == 'R';

        mWonAuto = switch (alliance.get()) {
            case Red  -> redWonAuto;
            case Blue -> !redWonAuto;
        };

        mOurHubActiveShift1 = (alliance.get() == Alliance.Red) ? !redWonAuto : redWonAuto;
        mAutoResultLocked = true;
    }

    /** Returns true if our alliance scored more fuel in auto. */
    public boolean wonAuto() {
        return mWonAuto;
    }

    // ── Shift tracking ─────────────────────────────────────────────────────

    public Shift getCurrentShift() {
        if (DriverStation.isAutonomousEnabled()) return Shift.AUTO;
        if (!DriverStation.isTeleopEnabled())    return Shift.DISABLED;

        double t = mTeleopTimer.get();

        if (t < TRANSITION_END) return Shift.TRANSITION;
        if (t < SHIFT_1_END)    return Shift.SHIFT_1;
        if (t < SHIFT_2_END)    return Shift.SHIFT_2;
        if (t < SHIFT_3_END)    return Shift.SHIFT_3;
        if (t < SHIFT_4_END)    return Shift.SHIFT_4;

        // If we won auto, END GAME is a continuous extension of our
        // active scoring window — report as SHIFT_4 so consumers see
        // one uninterrupted period
        if (mWonAuto)           return Shift.SHIFT_4;

        return Shift.ENDGAME;
    }

    public double getSecondsRemainingInShift() {
        if (!DriverStation.isTeleopEnabled()) return 0;

        double t    = mTeleopTimer.get();
        Shift shift = getCurrentShift();

        // Won auto: SHIFT_4 + END GAME = one 55s active window
        if (mWonAuto && (shift == Shift.SHIFT_4 || shift == Shift.ENDGAME)) {
            return Math.max(MATCH_END - t, 0);
        }

        return switch (shift) {
            case TRANSITION -> TRANSITION_END - t;
            case SHIFT_1    -> SHIFT_1_END    - t;
            case SHIFT_2    -> SHIFT_2_END    - t;
            case SHIFT_3    -> SHIFT_3_END    - t;
            case SHIFT_4    -> SHIFT_4_END    - t;
            case ENDGAME    -> Math.max(MATCH_END - t, 0);
            default         -> 0;
        };
    }

    // ── Hub status ─────────────────────────────────────────────────────────

    public boolean isHubActive() {
        if (!DriverStation.isTeleopEnabled())    return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!mAutoResultLocked)                  return true;

        Shift shift = getCurrentShift();

        if (shift == Shift.TRANSITION || shift == Shift.ENDGAME) return true;

        // Won auto: SHIFT_4 past SHIFT_4_END elapsed is really END GAME — both active
        if (mWonAuto && shift == Shift.SHIFT_4
                && mTeleopTimer.get() >= SHIFT_4_END) return true;

        return switch (shift) {
            case SHIFT_1 ->  mOurHubActiveShift1;
            case SHIFT_2 -> !mOurHubActiveShift1;
            case SHIFT_3 ->  mOurHubActiveShift1;
            case SHIFT_4 -> !mOurHubActiveShift1;
            default      -> true;
        };
    }

    // ── Rumble: shift ending warning ───────────────────────────────────────

    public boolean shouldRumbleShiftWarning() {
    return mShouldRumbleWarning;
    }

    public boolean shouldRumbleShiftStart() {
    return mShouldRumbleStart;
    }
    // ── Periodic ───────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        tryLockAutoResult();

        // Detect shift change once per cycle — used by both rumble methods
        Shift currentShift = getCurrentShift();
        mShiftChanged = currentShift != mLastShift;
        if (mShiftChanged) {
            mLastShift         = currentShift;
            mShiftWarningFired = false;
            mShiftStartFired   = false;
        }

        
        mShouldRumbleWarning = false;
        mShouldRumbleStart = false;

        if (currentShift != Shift.DISABLED
            && currentShift != Shift.AUTO
            && currentShift != Shift.ENDGAME) {

        double remaining = getSecondsRemainingInShift();

        if (!mShiftWarningFired && remaining <= SHIFT_WARNING_THRESHOLD && remaining > 0) {
            mShiftWarningFired = true;
            mShouldRumbleWarning = true;
        }
        }

        if (currentShift != Shift.DISABLED
            && currentShift != Shift.AUTO
            && currentShift != Shift.TRANSITION) {

        if (!mShiftStartFired && mShiftChanged) {
            mShiftStartFired = true;
            mShouldRumbleStart = true;
        }
        }

        DogLog.log("MatchState/SecondsRemainingInShift", getSecondsRemainingInShift());
        DogLog.log("MatchState/WonAuto",                 mWonAuto);
        DogLog.log("MatchState/HubActive",               isHubActive());
        DogLog.log("MatchState/CurrentShift",            currentShift.toString());
        DogLog.log("MatchState/ShiftWarning",            shouldRumbleShiftWarning());
        DogLog.log("MatchState/ShiftStart",              shouldRumbleShiftStart());
    }

}