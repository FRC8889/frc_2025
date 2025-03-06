package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        public static final int kFLDriveID = 1;
        public static final int KFLTurningID = 1;
        public static final int kFLEncoderID = 1;
        public static final double kFLEncoderOffset = -0.35 + Math.PI;
        public static final int kFLEncoderReversed = 1;

        public static final int kFRDriveID = 2;
        public static final int KFRTurningID = 2;
        public static final int kFREncoderID = 2;
        public static final double kFREncoderOffset = 0.65;
        public static final int kFREncoderReversed = 1;

        public static final int kBLDriveID = 4;
        public static final int KBLTurningID = 4;
        public static final int kBLEncoderID = 4;
        public static final double kBLEncoderOffset = -0.35;
        public static final int kBLEncoderReversed = 1;

        public static final int kBRDriveID = 3;
        public static final int KBRTurningID = 3;
        public static final int kBREncoderID = 3;
        public static final double kBREncoderOffset = 0.35 + Math.PI;
        public static final int kBREncoderReversed = 1;

    }

    public static final class OperatorConstants {

        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotationAxis = 4;
        public static final int kDriverFieldOrientedButtonId = 6;
        
        public static final double kDeadband = 0.08;
        
        public static final double kMaxDriveSpeedMetersPerSecond = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3;
        public static final double kMaxDriveAcceleration = 2;
        public static final double kMaxAngularAcceleration = 3.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = .25;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            OperatorConstants.kMaxAngularSpeedRadiansPerSecond, OperatorConstants.kMaxAngularAcceleration);
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 1;

    }

    public static final class ManipulatorConstants {

        public static final int kElevatorMotor1ID = 5;
        public static final int kElevatorMotor2ID = 6;
        public static final int kClawMotorID = 7;
        public static final int kIntakeMotorID = 8;

        public static final double kElevatorPositionIntake = 0.5;
        public static final double kElevatorPositionL1 = 3;
        public static final double kElevatorPositionL2 = 18;
        public static final double kElevatorPositionL3 = 46;
        public static final double kElevatorPositionL4 = 107;
        public static final double kElevatorPositionAlgaeProcesser = 16;
        public static final double kElevatorPositionAlgaeLow = 36;
        public static final double kElevatorPositionAlgaeHigh = 65;
        

        public static final double kClawPositionFallback = 1.9;
        public static final double kClawPositionIntake = 0.75;
        public static final double kClawPositionL1 = 0.5;
        public static final double kClawPositionL2 = 3.5;
        public static final double kClawPositionL3 = 3.5;
        public static final double kClawPositionL4 = 7;
        public static final double kClawPositionAlgaeProcesser = 15;
        public static final double kClawPositionAlgaeLow = 13;
        public static final double kClawPositionAlgaeHigh = 13;

        // Claw safe values
        public static final double kClawPositionSafe1Min = 1.5;
        public static final double kClawPositionSafe1Max = 2;
        public static final double kClawPositionSafe2Min = 5.5;
        public static final double kClawPositionSafe2Max = 20.0;

    }


}
