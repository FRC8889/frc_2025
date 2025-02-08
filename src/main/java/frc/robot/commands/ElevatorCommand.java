package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(0.03, 0.001, 0); // PID values, tune as necessary
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        pidController.reset();
        System.out.println("ElevatorCommand initialized");
    }

    @Override
    public void execute() {
        // Get setpoint and current encoder position
        double setpoint = elevatorSubsystem.getCurrentSetpoint();
        double currentPosition = elevatorSubsystem.getEncoder();

        // Calculate speed using PID
        double speed = pidController.calculate(currentPosition, setpoint);

        // Clamp speed to safe range
        speed = Math.max(-0.5, Math.min(0.5, speed));

        // Set motor speed
        elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotor();
        System.out.println("ElevatorCommand ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Command runs until explicitly stopped
    }
}
