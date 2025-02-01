package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.428;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kCANcoderRot2Rad = 6.283185;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRpm2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        public static final int kPhysicalMaxSpeedMetersPerSecond = 3;

        // Distance right and left
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        // Distance front and back
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //
        );

        public static final int kFLDriveID = 2;
        public static final int KFLTurningID = 1;
        public static final int kFLEncoderID = 1;
        public static final double kFLEncoderOffset = 0.0;
        public static final int kFLEncoderReversed = 0;

        public static final int kFRDriveID = 4;
        public static final int KFRTurningID = 3;
        public static final int kFREncoderID = 2;
        public static final double kFREncoderOffset = 0.0;
        public static final int kFREncoderReversed = 0;

        public static final int kBLDriveID = 6;
        public static final int KBLTurningID = 5;
        public static final int kBLEncoderID = 3;
        public static final double kBLEncoderOffset = 0.0;
        public static final int kBLEncoderReversed = 0;

        public static final int kBRDriveID = 8;
        public static final int KBRTurningID = 7;
        public static final int kBREncoderID = 4;
        public static final double kBREncoderOffset = 0.0;
        public static final int kBREncoderReversed = 0;

    }

    public static final class OConstants {

        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotationAxis = 4;
        public static final int kDriverFieldOrientedButtonId = 1;
        
        public static final double kDeadband = 0.08;
        
        public static final double kMaxDriveSpeedMetersPerSecond = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3;
        public static final double kMaxDriveAcceleration = 5;
        public static final double kMaxAngularAcceleration = 5;
    }


}
