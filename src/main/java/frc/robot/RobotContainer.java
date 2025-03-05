// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ManuipulatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
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

        configureBindings();
        elevatorSubsystem.setDefaultCommand(new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem));
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> swerveSubsystem.resetEncoders()));
        

        //new JoystickButton(driverJoystick, 1).whileTrue(new InstantCommand(() -> elevatorSubsystem.increaseElevatorStage(), elevatorSubsystem));
        //new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> intakeSubsystem.collectAlgaeOrOuttakeCoral(), intakeSubsystem));
        //new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> intakeSubsystem.outtakeAlgae(), intakeSubsystem));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
