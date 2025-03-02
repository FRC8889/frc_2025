package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;



public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(0.03, 0.0001, 0); // PID values, tune as necessary
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        pidController.reset();
        System.out.println("ElevatorCommand initialized");
    }

    @Override
    public void execute() {
       // elevatorSubsystem.elevatorstagecomplete = false;


        // Get setpoint and current encoder position
        double Elevatorsetpoint = elevatorSubsystem.getElevatorCurrentSetpoint();
        double ElevatorcurrentPosition = elevatorSubsystem.getElevator1Encoder();

        // double ClawAngleSetpoint = elevatorSubsystem.getClawAngleCurrentSetpoint();
        // double ClawAnglecurrentPosition = elevatorSubsystem.getClawAngleEncoder();



        // Calculate speed using PID
        // double ElevatorSpeed = pidController.calculate(ElevatorcurrentPosition, Elevatorsetpoint);
        // double ClawAngleSpeed = pidController.calculate(ClawAnglecurrentPosition, ClawAngleSetpoint);
        double ElevatorSpeed = 0.02 * (Elevatorsetpoint - ElevatorcurrentPosition);
        // double ClawAngleSpeed = 0.02 * (ClawAngleSetpoint - ClawAnglecurrentPosition);

        // Clamp speed to safe range
        ElevatorSpeed = Math.max(-0.05, Math.min(0.05, ElevatorSpeed));
        // ClawAngleSpeed = Math.max(-0.025, Math.min(0.025, ClawAngleSpeed));
    

        // Set motor speed
        elevatorSubsystem.setElevatorMotor(ElevatorSpeed);
        // elevatorSubsystem.setClawAngleMotor(ClawAngleSpeed);

    //     if (ClawAnglecurrentPosition == elevatorSubsystem.getClawAngleCurrentSetpoint() && ElevatorSpeed == pidController.calculate(ElevatorcurrentPosition, Elevatorsetpoint ))
    //     {
    //         elevatorSubsystem.elevatorstagecomplete = true;
    //     }
    //     else
    //     {
    //         elevatorSubsystem.elevatorstagecomplete = false; 
    //     }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotor();
        System.out.println("ElevatorCommand ended");
    }

    @Override
    public boolean isFinished() {
   // Only run in teleop; finish immediately otherwise.
        return !DriverStation.isTeleop();   
     }
}
