package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turningMotor;

    private final Double driveEncoder;
    private final Double turningEncoder;

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

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0.0075);        
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }
    
    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRpm2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        //temporary until we have cancoders
        return cancoder.getAbsolutePosition().getValueAsDouble() * ModuleConstants.kCANcoderRot2Rad;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        //Convert the thing from rads to rotations before you set it because talons are dumb
        //Would be "getAbsoluteEncoderRad() / 6.283185" instead of 0 with cancoders
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad() / ModuleConstants.kCANcoderRot2Rad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

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

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
