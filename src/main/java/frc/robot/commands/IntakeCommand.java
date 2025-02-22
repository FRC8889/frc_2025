package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.collectCoral(); // Always try to collect coral
    }

    @Override
    public boolean isFinished() {
        return false; // Runs indefinitely until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake(); // Stop intake when command ends
    }
}
