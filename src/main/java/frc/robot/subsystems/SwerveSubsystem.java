package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    
    public final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
    
    private Pigeon2 gyro = new Pigeon2(1);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0), modulePositions);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    /** Resets gyro heading. */
    public void zeroHeading() {
        gyro.reset();
    }

    /** Returns gyro heading. */
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360);
    }

    /** Returns gyro heading as Rotation2d. */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    @Override
    public void periodic() {
        modulePositions[0] = frontLeft.getPosition();
        modulePositions[1] = frontRight.getPosition();
        modulePositions[2] = backLeft.getPosition();
        modulePositions[3] = backRight.getPosition();

        odometer.update(getRotation2d(), modulePositions);
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        frontLeft.PrintCancoderValue();
        frontRight.PrintCancoderValue();
        backLeft.PrintCancoderValue();
        backRight.PrintCancoderValue();
    }

    /** Stops all swerve modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    /** Resets all swerve module encoders. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
 
}
