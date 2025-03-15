package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor1 = new SparkMax(5, MotorType.kBrushless);
    private final SparkMax elevatorMotor2 = new SparkMax(6, MotorType.kBrushless);
    private final PIDController elevatorPIDController;
    private boolean isGamePieceCoral = true; // true for coral, false for algae
    public double elevatorRunSpeed = 0;


    public ElevatorSubsystem() {
        // Initialize motors and pid for whatever (DO NOT SET THE MOTOR CONFIGS IN CODE OR YOU WILL BREAK THE ROBOT AGAIN BY ACCIDENT)
        // Do this in the spark's firmware to set stuff like current and braking mode -^^^
        elevatorPIDController = new PIDController(0.075, 0.001, 0.005);
        elevatorPIDController.reset();
    }

    public void ResetEncoders() {
        elevatorMotor1.getEncoder().setPosition(0);
        elevatorMotor2.getEncoder().setPosition(0);
        isGamePieceCoral = true;
    }

    public void ResetPid() {
        elevatorPIDController.reset();
    }

    @Override
    public void periodic() {
        // Display current stage and encoder value on SmartDashboard
        SmartDashboard.putNumber("Elevator Encoder", GetCollectiveElevatorEncoder());
        SmartDashboard.putBoolean("Elevator In Motion", InMotion());

    }

    /** Toggles game piece mode starting from Coral. */
    public void SwapLocalGamePiece() {
        isGamePieceCoral = !isGamePieceCoral;
    }

    public boolean InMotion() {
        if (Math.abs(elevatorRunSpeed) < 0.1) {
            return false;
        } else {
            return true;
        }
    }
    
    public boolean AtTarget() {
        if (6 > Math.abs(elevatorPIDController.getSetpoint() - GetCollectiveElevatorEncoder())) {
            return true;
        } else {
            return false;
        }
    }    

    public double GetElevator1Encoder() {
        return elevatorMotor1.getEncoder().getPosition();
    }
    public double GetElevator2Encoder() {
        return elevatorMotor2.getEncoder().getPosition();
    }
    /** Returns average elevator encoder readings. */
    public double GetCollectiveElevatorEncoder() {
        return (-GetElevator1Encoder() + GetElevator2Encoder()) / 2;
    }

    public double GetCollectiveElevatorCurrent() {
        return (elevatorMotor1.getOutputCurrent() + elevatorMotor2.getOutputCurrent()) / 2;
    }

    /** Set Elevator target position. */
    public void SetElevatorTargetPosition(double position) {
        elevatorPIDController.setSetpoint(position);
    }

    /** Runs Elevator PID controller and sets speed. */
    public void RunElevatorPID() {
        double ElevatorcurrentPosition = GetCollectiveElevatorEncoder();
        double ElevatorSpeed = elevatorPIDController.calculate(ElevatorcurrentPosition);
        SetElevatorMotors(ElevatorSpeed);
        elevatorRunSpeed = ElevatorSpeed;
        if (ElevatorSpeed > -0.7) {
            SetElevatorMotors(ElevatorSpeed);
        } else {
            SetElevatorMotors(-0.7);
        }
    }

    /** Set Elevator motor speed. */
    public void SetElevatorMotors(double speed) {
        elevatorMotor1.set(-speed);
        elevatorMotor2.set(speed);
        SmartDashboard.putNumber("elevator speed", speed);

    }

    /** Stop the Elevator and Claw motor. */
    public void StopMotors() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }
}
