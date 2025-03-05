package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turningMotor;

    private Double driveEncoder;
    private Double turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder cancoder;
    private final int absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, int cancoderid, double absoluteEncoderOffset, int absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        cancoder = new CANcoder(cancoderid);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getPosition().getValueAsDouble();
        turningEncoder = turningMotor.getEncoder().getPosition();

        turningPidController = new PIDController(0.5, 0.001, 0.0075);        
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        
    }

    /** Returns drive motor encoder position in meters. */
    public double getDrivePosition() {
        return driveEncoder * ModuleConstants.kDriveEncoderRot2Meter;
    }

    /** Returns turning motor encoder position in radians. */
    public double getTurningPosition() {
        return turningEncoder * ModuleConstants.kTurningEncoderRot2Rad;
    }

    /** Returns drive motor velocity in meters per second. */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }
    
    /** Returns turning motor velocity in radians per second. */
    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRpm2RadPerSec;
    }

    /** Returns adjusted absolute encoder position in radians. */
    public double getAbsoluteEncoderRad() {
        return absoluteEncoderOffsetRad + (cancoder.getAbsolutePosition().getValueAsDouble() * -absoluteEncoderReversed * ModuleConstants.kCANcoderRot2Rad);
    }

    /** Resets motor encoders and calibrates steering motor encoder with absolute encoder. */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.getEncoder().setPosition(21.428 * getAbsoluteEncoderRad() / ModuleConstants.kCANcoderRot2Rad);
    }

    /** Returns current module state. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** Sets desired module state. */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + cancoder.getDeviceID() + "] state", state.toString() + getAbsoluteEncoderRad());
    }

    public void PrintCancoderValue() {
        SmartDashboard.putNumber("Cancoder[" + cancoder.getDeviceID() + "] radians", getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }


    /** Stops drive and turning motors. */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
