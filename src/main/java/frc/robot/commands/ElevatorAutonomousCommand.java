package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAutonomousCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double targetElevatorPosition;
  private final double targetClawAngle;

  // Create ProfiledPIDControllers with sample gains and motion constraints
  private final ProfiledPIDController elevatorController;
  private final ProfiledPIDController clawController;
  
  public ElevatorAutonomousCommand(ElevatorSubsystem elevatorSubsystem, double targetElevatorPosition, double targetClawAngle) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.targetElevatorPosition = targetElevatorPosition;
    this.targetClawAngle = targetClawAngle;
    
    // Define trapezoidal profile constraints (adjust as needed)
    TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(1.0, 0.5);
    TrapezoidProfile.Constraints clawConstraints = new TrapezoidProfile.Constraints(1.0, 0.5);
    
    elevatorController = new ProfiledPIDController(0.03, 0, 0, elevatorConstraints);
    clawController = new ProfiledPIDController(0.03, 0, 0, clawConstraints);
    
    // Set tolerances for when the command should finish
    elevatorController.setTolerance(0.5);
    clawController.setTolerance(0.5);
    
    addRequirements(elevatorSubsystem);
  }
  
  @Override
  public void initialize() {
    // Reset controllers to start at the current positions
    elevatorController.reset(elevatorSubsystem.getElevator1Encoder());
    clawController.reset(elevatorSubsystem.getClawAngleEncoder());
  }
  
  @Override
  public void execute() {
    // Compute the control outputs
    double elevatorOutput = elevatorController.calculate(elevatorSubsystem.getElevator1Encoder(), targetElevatorPosition);
    double clawOutput = clawController.calculate(elevatorSubsystem.getClawAngleEncoder(), targetClawAngle);
    
    // Optionally, clamp the outputs if needed:
    elevatorOutput = Math.max(-0.5, Math.min(0.5, elevatorOutput));
    clawOutput = Math.max(-0.25, Math.min(0.25, clawOutput));
    
    // Set motor outputs
    elevatorSubsystem.setElevatorMotor(elevatorOutput);
    elevatorSubsystem.setClawAngleMotor(clawOutput);
  }
  
  @Override
  public boolean isFinished() {
    // Command finishes when both controllers are within tolerance of their goals
    return elevatorController.atGoal() && clawController.atGoal();
  }
  
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMotor();
  }
}
