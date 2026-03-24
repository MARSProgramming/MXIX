package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/**
 * Tracks match state for FRC Rebuilt (2026).
 *
 * Hub shift timing (seconds remaining in match timer):
 *   TRANSITION SHIFT: 2:20 – 2:10 (both hubs active)
 *   SHIFT 1:          2:10 – 1:45
 *   SHIFT 2:          1:45 – 1:20
 *   SHIFT 3:          1:20 – 0:55
 *   SHIFT 4:          0:55 – 0:30
 *   END GAME:         0:30 – 0:00 (both hubs active)
 *
 * Auto winner's hub is INACTIVE in SHIFT 1.
 
 * ═══════════════════════════════════════════════════════════════
 * TESTING MatchStateSubsystem WITHOUT FMS (Practice Matches)
 * ═══════════════════════════════════════════════════════════════
 *
 * In a real match, FMS automatically sends game data indicating
 * which alliance won auto. On a practice field without FMS, you
 * must set this manually in the FRC Driver Station application:
 *
 *   1. Open the FRC Driver Station
 *   2. Navigate to the "Setup" tab
 *   3. Find the "Game Data" text field
 *   4. Enter one of the following:
 *        "R" → Red alliance won auto (Red hub inactive in Shift 1)
 *        "B" → Blue alliance won auto (Blue hub inactive in Shift 1)
 *   5. Enable the robot and run a practice match as normal
 *
 * Leaving the Game Data field empty simulates no FMS connection,
 * in which case isHubActive() will always return true.
 *
 * Verify correct behavior in DogLog:
 *   - MatchState/WonAuto        → locks in at teleop start
 *   - MatchState/HubActive      → flips correctly each shift
 *   - MatchState/CurrentShift   → transitions at correct times
 * ═══════════════════════════════════════════════════════════════
 */

public class MatchStateSubsystem extends SubsystemBase {

    // ── Shift boundaries (seconds remaining in match timer) ───────────────
    private static final double SHIFT_1_START = 130; // 2:10
    private static final double SHIFT_2_START = 105; // 1:45
    private static final double SHIFT_3_START =  80; // 1:20
    private static final double SHIFT_4_START =  55; // 0:55
    private static final double ENDGAME_START =  30; // 0:30

    // ── Auto result ────────────────────────────────────────────────────────
    private boolean mWonAuto          = false;
    private boolean mAutoResultLocked = false;

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

        double t = DriverStation.getMatchTime();

        if (t > SHIFT_1_START) return Shift.TRANSITION;
        if (t > SHIFT_2_START) return Shift.SHIFT_1;
        if (t > SHIFT_3_START) return Shift.SHIFT_2;
        if (t > SHIFT_4_START) return Shift.SHIFT_3;
        if (t > ENDGAME_START) return Shift.SHIFT_4;
        return Shift.ENDGAME;
    }

    public double getSecondsRemainingInShift() {
        if (!DriverStation.isTeleopEnabled()) return 0;

        double t = DriverStation.getMatchTime();

        return switch (getCurrentShift()) {
            case TRANSITION -> t - SHIFT_1_START;
            case SHIFT_1    -> t - SHIFT_2_START;
            case SHIFT_2    -> t - SHIFT_3_START;
            case SHIFT_3    -> t - SHIFT_4_START;
            case SHIFT_4    -> t - ENDGAME_START;
            case ENDGAME    -> t;
            default         -> 0;
        };
    }

    // ── Hub status ─────────────────────────────────────────────────────────

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled())    return false;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        Shift shift = getCurrentShift();

        if (shift == Shift.TRANSITION || shift == Shift.ENDGAME) return true;

        boolean redInactiveFirst = gameData.charAt(0) == 'R';
        boolean weAreRed = alliance.get() == Alliance.Red;

        boolean ourHubActiveInShift1 = weAreRed ? !redInactiveFirst : redInactiveFirst;

        return switch (shift) {
            case SHIFT_1 ->  ourHubActiveInShift1;
            case SHIFT_2 -> !ourHubActiveInShift1;
            case SHIFT_3 ->  ourHubActiveInShift1;
            case SHIFT_4 -> !ourHubActiveInShift1;
            default      -> true;
        };
    }

    // ── Periodic ───────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        tryLockAutoResult();

        DogLog.log("MatchState/SecondsRemainingInShift", getSecondsRemainingInShift());
        DogLog.log("MatchState/WonAuto",                 mWonAuto);
        DogLog.log("MatchState/HubActive",               isHubActive());
        DogLog.log("MatchState/CurrentShift",            getCurrentShift().toString());
    }
}