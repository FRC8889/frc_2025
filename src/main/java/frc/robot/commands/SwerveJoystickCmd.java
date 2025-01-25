package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(OConstants.kMaxDriveAcceleration);
        this.yLimiter = new SlewRateLimiter(OConstants.kMaxDriveAcceleration);
        this.turningLimiter = new SlewRateLimiter(OConstants.kMaxAngularAcceleration);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        // Get momentary joystick values
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        
        // Deadband
        xSpeed = Math.abs(xSpeed) > OConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OConstants.kDeadband ? turningSpeed : 0;
       
        // Rate & speed limit
        xSpeed = xLimiter.calculate(xSpeed) * OConstants.kMaxDriveSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * OConstants.kMaxDriveSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * OConstants.kMaxAngularSpeedRadiansPerSecond;
        
        // Create speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Field oriented
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Robot oriented
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // Turn speeds to module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
