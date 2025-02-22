package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor1 = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax elevatorMotor2 = new SparkMax(5, MotorType.kBrushless);
    private final SparkMax ClawAngleMotor = new SparkMax(8, MotorType.kBrushless);
    private final double[] elevatorsetpoints = {10, 20, 30, 40}; // Setpoints for each Elevator stage
    private final double[] ClawAngleSetpoints = {10, 30, 30, 40};  // Setpoints for each claw position
    private int elevatorStage = 1; // Current stage (1 to 4)
    public boolean elevatorstagecomplete = true;
    

    public ElevatorSubsystem() {
        // Initialize motor if needed, like setting idle mode
    }

    @Override
    public void periodic() {
        // Display current stage and encoder value on SmartDashboard
        SmartDashboard.putNumber("Elevator Stage", elevatorStage);
        SmartDashboard.putNumber("ElevatorMotor1 Encoder", getElevator1Encoder());
        SmartDashboard.putNumber("ElevatorMotor2 Encoder", getElevator2Encoder());
        SmartDashboard.putNumber("Claw Angle", getClawAngleEncoder());
        SmartDashboard.putNumber("setpoint", getElevatorCurrentSetpoint());
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

    
    public double getClawAngleCurrentSetpoint() {
        // Update flipper motor based on stage
       return ClawAngleSetpoints[elevatorStage - 1];
    }

    /**
     * Get the current setpoint based on the elevator stage.
     * 
     * @return Setpoint for the current stage.
     */
    public double getElevatorCurrentSetpoint() {
        return elevatorsetpoints[elevatorStage - 1]; // Index setpoints array
    }

    /**
     * Get the current encoder position.
     * 
     * @return The encoder position from the motor.
     */
    public double getElevator1Encoder() {
        return elevatorMotor1.getEncoder().getPosition();
    }

    public double getElevator2Encoder() {
        return elevatorMotor2.getEncoder().getPosition();
    }

    public double getClawAngleEncoder() {
        return ClawAngleMotor.getEncoder().getPosition();
    }
    /**
     * Set the motor speed.
     * 
     * @param speed The speed to set the motor (-1.0 to 1.0).
     */
    public void setElevatorMotor(double speed) {
        elevatorMotor1.set(speed);
        elevatorMotor2.set(-speed);

    }

    public void setClawAngleMotor(double speed) {
        ClawAngleMotor.set(speed);
    }

    /**
     * Stop the Elevator and Claw motor.
     */
    public void stopMotor() {
        elevatorMotor1.set(0);
        ClawAngleMotor.set(0);
    }




}
