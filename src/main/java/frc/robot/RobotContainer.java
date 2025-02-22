// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final Joystick driverJoystick = new Joystick(OConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OConstants.kDriverRotationAxis),
            () -> !driverJoystick.getRawButton(OConstants.kDriverFieldOrientedButtonId)
        ));


           // Set ElevatorCommand as the default command for the ElevatorSubsystem
           elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem));

        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, 2).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading()));

  
        // Button X (on playstation the best controller) changes the stage
        new JoystickButton(driverJoystick, 1).debounce(0.1).whileTrue(new RunCommand(() -> elevatorSubsystem.increaseStage(), elevatorSubsystem));
          
        // Button L1 (on playstation the best controller) picks up coral(not really) spits out algae
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> intakeSubsystem.collectAlgaeOrOuttakeCoral(), intakeSubsystem));

        // Button R1 (on playstation the best controller) spits out coral and intakes in algae
        new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> intakeSubsystem.outtakeAlgae(), intakeSubsystem));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
