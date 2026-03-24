package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Settings;
import frc.robot.constants.SystemConstants.Drive;
import frc.robot.subsystems.Cowl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.ManualDriveInput;
import frc.robot.util.ShotSetup;

public class AimAndShuttle extends Command {

    private boolean mReadyToShoot = false;

    private final Swerve          swerve;
    private final Cowl            cowl;
    private final Flywheel        flywheel;
    private final Feeder          feeder;
    private final Floor           floor;
    private final IntakeRollers   intakeRollers;
    private final ShotSetup       shotSetup;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle facingAngleRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Drive.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Drive.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0);

    public AimAndShuttle(
        Swerve swerve, Cowl cowl, Flywheel flywheel, Feeder feeder,
        Floor floor, IntakeRollers intakeRollers, ShotSetup shotSetup,
        DoubleSupplier forwardInput, DoubleSupplier leftInput
    ) {
        this.swerve        = swerve;
        this.cowl          = cowl;
        this.flywheel      = flywheel;
        this.feeder        = feeder;
        this.floor         = floor;
        this.intakeRollers = intakeRollers;
        this.shotSetup     = shotSetup;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve, cowl, flywheel, feeder, floor, intakeRollers);
    }

    @Override
    public void initialize() { mReadyToShoot = false; }

    @Override
    public void execute() {
        ManualDriveInput   input = inputSmoother.getSmoothedInput();
        ShotSetup.SOTMInfo sotm  = shotSetup.getSOTMInfoShuttle(swerve);

        double cowlAngle = sotm.shotInfo.cowlPosition;
        double rpm       = sotm.shotInfo.shot.shooterRPM;

        swerve.setControl(facingAngleRequest
            .withVelocityX(Drive.kMaxSpeed.times(input.forward))
            .withVelocityY(Drive.kMaxSpeed.times(input.left))
            .withTargetDirection(sotm.virtualTargetAngle)
            .withTargetRateFeedforward(RadiansPerSecond.of(sotm.angularVelocityRadPerSec)));

        cowl.setPosition(cowlAngle);
        flywheel.setRPM(rpm);

        DogLog.log("AimAndShuttle/CowlTarget",  cowlAngle);
        DogLog.log("AimAndShuttle/RPMTarget",    rpm);
        DogLog.log("AimAndShuttle/VirtualAngle", sotm.virtualTargetAngle.getDegrees());
        DogLog.log("AimAndShuttle/AngularVelFF", sotm.angularVelocityRadPerSec);

        if (swerve.isAimedAtVirtualTarget(sotm.virtualTargetAngle)
                && flywheel.isVelocityWithinTolerance(RPM.of(rpm))) {
            mReadyToShoot = true;
        }

        if (mReadyToShoot) {
            feeder.setPercentOut(Settings.FeedSystemSettings.FEEDER_FEED_DUTYCYCLE);
            intakeRollers.setPercentOut(Settings.FeedSystemSettings.INTAKEROLLER_FEED_DUTYCYCLE);
            floor.setPercentOut(Settings.FeedSystemSettings.FLOOR_FEED_DUTYCYCLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setPercentOut(0);
        floor.setPercentOut(0);
        flywheel.setRPM(0);
        intakeRollers.setPercentOut(0);
    }

    @Override
    public boolean isFinished() { return false; }
}