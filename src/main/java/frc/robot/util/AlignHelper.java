package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/**
 * @author Alexander Szura, 5409 CHARGERS
 */
public class AlignHelper {
    /**
     * Calculates rotation difference for rotation2d
     * @param r1 1st rotation2d
     * @param r2 2nd rotation2d
     * @return The angle between them
     */
    public static Angle rotationDifference(Rotation2d r1, Rotation2d r2) {
        double difference = r1.getRadians() - r2.getRadians();

        difference = (difference + Math.PI) % (2 * Math.PI) - Math.PI;

        while (difference < -Math.PI)
            difference += 2 * Math.PI;

        while (difference > Math.PI)
            difference -= 2 * Math.PI;

        return Radians.of(Math.abs(difference));
    }
}