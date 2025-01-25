package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RandomNeo;

public class PidNeoCmd extends Command {
    private final RandomNeo randomNeo;
    private final PIDController pidController;
    
    public PidNeoCmd(RandomNeo randomNeo, double setpoint) {
        this.randomNeo = randomNeo;
        this.pidController = new PIDController(0.03, 0.001, 0);
        pidController.setSetpoint(setpoint);
        addRequirements(randomNeo);
    }

    @Override
    public void initialize() {
        pidController.reset();
        System.out.println("pidNeoCmd initialized");
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(randomNeo.getEncoder());
        randomNeo.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        randomNeo.setMotor(0);
        System.out.println("pidNeoCmd killed");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
