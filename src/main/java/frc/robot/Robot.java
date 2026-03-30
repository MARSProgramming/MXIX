// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    // Loop timing monitoring
    private double lastPeriodicTime = 0;
    private static final double LOOP_OVERRUN_THRESHOLD_SECONDS = 0.025; // 25ms threshold for warning
    private static final double CRITICAL_LOOP_OVERRUN_THRESHOLD_SECONDS = 0.030; // 30ms for critical warning
    private final Alert loopOverrunAlert =
        new Alert("Loop overrun detected! Check Driver Station Log for details.", AlertType.kWarning);
    private final Alert criticalLoopOverrunAlert =
        new Alert("CRITICAL loop overrun! Robot performance degraded.", AlertType.kError);
    private int loopOverrunCount = 0;
    private static final int LOOP_OVERRUN_ALERT_THRESHOLD = 10; // Alert after 10 overruns
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        //RobotController.setBrownoutVoltage(Volts.of(6.1));

        // Initialize loop timing
        lastPeriodicTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }


    @Override
    public void robotInit() {
        m_robotContainer.getLedSubsystem().setColor(Color.kRed, LEDSegment.ALL);
    }

    @Override
    public void testInit() {
        m_robotContainer.configureTestBindings();
    }
    

    @Override
    public void teleopInit() {
        m_robotContainer.getMatchStateSubsystem().onTeleopInit();
    }
    


    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * 
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */

    @Override
    public void robotPeriodic() {
        // Measure loop time for performance monitoring
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double loopTime = currentTime - lastPeriodicTime;

        // Check for loop overruns
        if (loopTime > CRITICAL_LOOP_OVERRUN_THRESHOLD_SECONDS) {
            loopOverrunCount++;
            criticalLoopOverrunAlert.set(true);
            DriverStation.reportError(
                String.format("CRITICAL loop overrun: %.2f ms (threshold: %.2f ms)",
                    loopTime * 1000, CRITICAL_LOOP_OVERRUN_THRESHOLD_SECONDS * 1000),
                false);
        } else if (loopTime > LOOP_OVERRUN_THRESHOLD_SECONDS) {
            loopOverrunCount++;
            if (loopOverrunCount >= LOOP_OVERRUN_ALERT_THRESHOLD) {
                loopOverrunAlert.set(true);
            }
            DriverStation.reportWarning(
                String.format("Loop overrun: %.2f ms (threshold: %.2f ms)",
                    loopTime * 1000, LOOP_OVERRUN_THRESHOLD_SECONDS * 1000),
                false);
        } else {
            // Reset alerts if loop time is good
            if (loopOverrunCount > 0) {
                loopOverrunCount--;
            }
            if (loopOverrunCount == 0) {
                loopOverrunAlert.set(false);
                criticalLoopOverrunAlert.set(false);
            }
        }

        // Log loop time to DogLog for performance monitoring
        DogLog.log("Robot/LoopTimeMs", loopTime * 1000);
        DogLog.log("Robot/LoopOverrunCount", loopOverrunCount);

        lastPeriodicTime = currentTime;

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
}