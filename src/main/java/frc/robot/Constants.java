package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kAngleMotorGearRatio = 1 / 21.43;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kAngleEncoderRot2Meter = kAngleMotorGearRatio * Math.PI * 2;
        public static final double kDriveEncoderRPM2MPS = kDriveEncoderRot2Meter / 60;
        public static final double kAngleEncoderRPM2MPS = kAngleEncoderRot2Meter / 60;
        public static final double kP_Drive = 0.0020645;
        public static final double kP_Angle = 0.01;
    }

    public static final class DriveConstants {
        public static final double kSpeedLimiter = .4;

        public static final double kRobotMaxSpeed = 1.3716;

        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        public static final double kWheelBase = Units.inchesToMeters(26.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        public static final int kFrontLeftDriveId = 11;
        public static final int kFrontLeftAngleId = 12;
        public static final int kFrontLeftAbsoluteEncoderId = 21;
        public static final double kFrontLeftAbsoluteEncoderOffset = Units.degreesToRadians(19.688);// + MathConstants.HALFPI;
        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kFrontLeftAngleReversed = true;
        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;

        public static final int kFrontRightDriveId = 7;
        public static final int kFrontRightAngleId = 8;
        public static final int kFrontRightAbsoluteEncoderId = 22;
        public static final double kFrontRightAbsoluteEncoderOffset = Units.degreesToRadians(8.261);// + MathConstants.HALFPI;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kFrontRightAngleReversed = true;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;

        public static final int kBackLeftDriveId = 13;
        public static final int kBackLeftAngleId = 14;
        public static final int kBackLeftAbsoluteEncoderId = 23;
        public static final double kBackLeftAbsoluteEncoderOffset = Units.degreesToRadians(235.986);// + MathConstants.HALFPI; // 235.986 19.5013
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kBackLeftAngleReversed = true;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;

        public static final int kBackRightDriveId = 5;
        public static final int kBackRightAngleId = 6;
        public static final int kBackRightAbsoluteEncoderId = 24;
        public static final double kBackRightAbsoluteEncoderOffset = Units.degreesToRadians(284.063);// + MathConstants.HALFPI; //284.063 11.5722
        public static final boolean kBackRightDriveReversed = true;
        public static final boolean kBackRightAngleReversed = true;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;
    }

    public static final class MathConstants {
        public static final double TWOPI = (2 * Math.PI);
        public static final double HALFPI = (Math.PI / 2);
    }

    public static final class TrackingConstants {
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
    }
}
