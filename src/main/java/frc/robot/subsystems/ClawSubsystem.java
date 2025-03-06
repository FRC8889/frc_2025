package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

    private final SparkMax clawMotor = new SparkMax(7, MotorType.kBrushless);
    private final PIDController clawPIDController;
    private boolean isGamePieceCoral = true; // true for coral, false for algae
    private int tracking = 0; 



    public ClawSubsystem() {
        // Initialize motors and pid for whatever (DO NOT SET THE MOTOR CONFIGS IN CODE OR YOU WILL BREAK THE ROBOT AGAIN BY ACCIDENT)
        // Do this in the spark's firmware to set stuff like current and braking mode -^^^
        clawPIDController = new PIDController(0.075, 0.05, 0.005);
        clawPIDController.reset();

    }

    public void ResetPid() {
        clawPIDController.reset();
    }

    public void SwapLocalGamePiece() {
        isGamePieceCoral = !isGamePieceCoral;
    }

    @Override
    public void periodic() {
        // Display encoder value on SmartDashboard
        SmartDashboard.putNumber("Claw Encoder", GetClawEncoder());
        SmartDashboard.putNumber("tracking", tracking);
    }
    
    /** Returns claw encoder reading. */    
    public double GetClawEncoder() {
        return clawMotor.getEncoder().getPosition();
    }

    /** Set Claw target position. */
    public void SetClawTargetPosition(double position) {
        clawPIDController.setSetpoint(position);
    }

    /** Runs Claw PID controller and sets speed. */
    public void RunClawPID() {
        double ClawcurrentPosition = GetClawEncoder();
        double ClawSpeed = clawPIDController.calculate(ClawcurrentPosition);
        SetClawMotors(ClawSpeed);
    }

    /** Set Claw motor speed. */
    public void SetClawMotors(double speed) {
        clawMotor.set(speed);
    }

    /** Stops Claw motor. */
    public void StopMotors() {
        clawMotor.set(0);
    }

    public void Tracking() {
        tracking++;
    }

}
