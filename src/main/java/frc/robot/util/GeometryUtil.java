package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;

public final class GeometryUtil {
    public static boolean isNear(Rotation2d expected, Rotation2d actual, Angle tolerance) {
        final double expectedRadians = MathUtil.angleModulus(expected.getRadians());
        final double actualRadians = MathUtil.angleModulus(actual.getRadians());
        return MathUtil.isNear(expectedRadians, actualRadians, tolerance.in(Radians), -Math.PI, Math.PI);
    }

    public static boolean isInField(Pose2d pose) {
        return (pose.getX() >= 0 && pose.getX() <= 16.54 && pose.getY() >= 0 && pose.getY() <= 8.01);
    } 

    public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
    }


}