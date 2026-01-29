package frc.robot.constants;


import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    // Orientations to target
    public static class Orientations {
        public static ArrayList<Rotation2d> legalOrientations = new ArrayList<Rotation2d>();
        static {
            legalOrientations.add(new Rotation2d(Units.Degrees.of(45)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(135)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(225)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(315)));
        }

        /*
         * Return the closest diamond orientation for the robot
         */

        public static Rotation2d getClosestDiamond(Pose2d robotPose) {
            Rotation2d currRot = robotPose.getRotation();
            Rotation2d closest = legalOrientations.get(0);
            double minError = Math.abs(currRot.minus(closest).getRadians());

            for (Rotation2d candidate : legalOrientations) {
                double error = Math.abs(currRot.minus(candidate).getRadians());
                if (error < minError) {
                minError = error;
                closest = candidate;
                }
            }

            return closest;
        }
    }

    public static class Locations {

        /**
         * Helper method to return the position of the legal hub to score in
         **/
        public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Units.Inches.of(182.105), Units.Inches.of(158.845));
        }
            return new Translation2d(Units.Inches.of(469.115), Units.Inches.of(158.845));
        }
    }
}
