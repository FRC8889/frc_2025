package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LaunchAlgaeToBargeCmd extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private final Joystick driverJoystick = new Joystick(0);
    private boolean endCommand;

    public LaunchAlgaeToBargeCmd(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(elevatorSubsystem, clawSubsystem, intakeSubsystem);


    }

        
    public void initialize() {
        System.out.println("Activate elevator breaker");
        elevatorSubsystem.SetElevatorTargetPosition(94);
        clawSubsystem.SetClawTargetPosition(7.5);
        endCommand = false;
    }
    
    @Override
    public void execute() {
        if (elevatorSubsystem.GetCollectiveElevatorEncoder() > 75) {
            clawSubsystem.SetClawTargetPosition(0);
        }
        if (elevatorSubsystem.GetCollectiveElevatorEncoder() > 82.5) {
            intakeSubsystem.UpdateIntakeOutput();
        }
        if (elevatorSubsystem.AtTarget()) {
            endCommand = true;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.print("Elevator destroyed");
    }

    @Override
  public boolean isFinished() {
    JoystickButton ForceResetElevatorButton = new JoystickButton(driverJoystick, 4);
    // end if in coral mode or if emergency reset button is pressed or command finished as usual
    return clawSubsystem.isGamePieceCoral()|| ForceResetElevatorButton.getAsBoolean() || endCommand;
  }



}
