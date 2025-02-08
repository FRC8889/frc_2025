package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor = new SparkMax(9, MotorType.kBrushless);
    private final double[] setpoints = {10, 20, 30, 40}; // Setpoints for each stage
    private int elevatorStage = 1; // Current stage (1 to 4)
    

    public ElevatorSubsystem() {
        // Initialize motor if needed, like setting idle mode
    }

    @Override
    public void periodic() {
        // Display current stage and encoder value on SmartDashboard
        SmartDashboard.putNumber("Elevator Stage", elevatorStage);
        SmartDashboard.putNumber("Elevator Encoder", getEncoder());
    }

    /**
     * Increment the elevator stage. Wraps around to 1 if it exceeds 4.
     */
    public void increaseStage() {
        elevatorStage++;
        if (elevatorStage > 4) {
            elevatorStage = 1; // Loop back to stage 1
        }
    }

    /**
     * Get the current setpoint based on the elevator stage.
     * 
     * @return Setpoint for the current stage.
     */
    public double getCurrentSetpoint() {
        return setpoints[elevatorStage - 1]; // Index setpoints array
    }

    /**
     * Get the current encoder position.
     * 
     * @return The encoder position from the motor.
     */
    public double getEncoder() {
        return elevatorMotor.getEncoder().getPosition();
    }

    /**
     * Set the motor speed.
     * 
     * @param speed The speed to set the motor (-1.0 to 1.0).
     */
    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    /**
     * Stop the motor.
     */
    public void stopMotor() {
        elevatorMotor.set(0);
    }
}
