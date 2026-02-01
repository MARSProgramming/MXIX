package frc.robot.constants;


import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

/**
 * Container for field-related constants and utility methods.
 * Includes target locations and orientation helpers.
 */
public class FieldConstants {

    // Orientations to target
    /**
     * Helper class for determining robot orientations on the field.
     */
    public static class Orientations {
        /**
         * List of "diamond" orientations (intercardinal directions: 45, 135, 225, 315 degrees).
         * Useful for traversing bumps or locking heading in specific directions.
         */
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
        /**
         * Finds the closest "diamond" orientation (45, 135, 225, 315) to the robot's current heading.
         *
         * @param robotPose The current pose of the robot.
         * @return The closest Rotation2d from the legalOrientations list.
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

        /**
         * Calculates the direction from the robot to the Hub (Speaker/Target).
         * Adjusts for the driver's perspective (Red vs Blue alliance).
         *
         * @param sw The Swerve subsystem instance.
         * @return The Rotation2d representing the angle to the target in Operator Perspective.
         */
        public static Rotation2d getDirectionToHub(Swerve sw) {
            final Translation2d hubPosition = Locations.hubPosition();
            final Translation2d robotPosition = sw.getState().Pose.getTranslation();
            
            // Calculate angle in standard field coordinates (Blue Alliance origin)
            final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
            
            // Adjust for the driver's perspective (Red vs Blue alliance)
            final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(sw.getOperatorForwardDirection());
            
            return hubDirectionInOperatorPerspective;
        }

    }

    /**
     * Container for specific field locations.
     */
    public static class Locations {

        /**
         * Helper method to return the position of the legal hub to score in based on the current alliance.
         * 
         * @return The Translation2d of the target hub (Speaker).
         **/
        public static Translation2d hubPosition() {
            final Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
                // Blue Alliance Hub Position
                return new Translation2d(Units.Inches.of(182.105), Units.Inches.of(158.845));
            }
            // Red Alliance Hub Position
            return new Translation2d(Units.Inches.of(469.115), Units.Inches.of(158.845));
        }
    }
}
