package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFLDriveID,
        DriveConstants.KFLTurningID,
        DriveConstants.kFLEncoderID,
        DriveConstants.kFLEncoderOffset,
        DriveConstants.kFLEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFRDriveID,
        DriveConstants.KFRTurningID,
        DriveConstants.kFREncoderID,
        DriveConstants.kFREncoderOffset,
        DriveConstants.kFREncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBLDriveID,
        DriveConstants.KBLTurningID,
        DriveConstants.kBLEncoderID,
        DriveConstants.kBLEncoderOffset,
        DriveConstants.kBLEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBRDriveID,
        DriveConstants.KBRTurningID,
        DriveConstants.kBREncoderID,
        DriveConstants.kBREncoderOffset,
        DriveConstants.kBREncoderReversed);
    
    private Pigeon2 gyro = new Pigeon2(1);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
 
}
