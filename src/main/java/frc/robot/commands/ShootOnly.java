package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.constants.FieldConstants.Locations;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.LEDSegment;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

/**
 * Command that allows the driver to translate the robot field-centrically while the robot
 * automatically rotates to face a specific target (the Hub/Speaker).
 */
public class ShootOnly extends Command {
    private boolean readyToShootBoolean = false;

    private final Cowl cowl;
    private final Flywheel flywheel;

    private final double cowlInput;
    private final double velocityInput;


    // Request to drive field-centric while facing a specific angle
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Drive.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Drive.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);


    public ShootOnly(
        Cowl cowl,
        Flywheel flywheel,
        double velocityInput,
        double cowlInput
    ) {
        this.cowl = cowl;
        this.flywheel = flywheel;
        this.cowlInput = cowlInput;
        this.velocityInput = velocityInput;

        addRequirements(cowl, flywheel);
    }




    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        cowl.setPosition(cowlInput);
        flywheel.setRPM(velocityInput);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setRPM(0);
        cowl.setPercentOut(0);
        
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (e.g., button release)
        return false;
    }

}