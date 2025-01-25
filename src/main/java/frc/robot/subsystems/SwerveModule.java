package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private final TalonFX turningMotor;

    private final Double driveEncoder;
    private final Double turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final int absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, int absoluteEncoderId, double absoluteEncoderOffset, int absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);
        // Temporary "encoder" values until we get cancoders
        driveEncoder = driveMotor.getPosition().getValueAsDouble();
        turningEncoder = turningMotor.getPosition().getValueAsDouble();

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0.0075);        
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }
    
    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble() * ModuleConstants.kTurningEncoderRpm2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        //temporary until we have cancoders
        return -turningMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        //Convert the thing from rads to rotations before you set it because talons are dumb
        //Would be "getAbsoluteEncoderRad() / 6.283185" instead of 0 with cancoders
        turningMotor.setPosition(0);
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
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
