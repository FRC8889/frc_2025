// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OConstants;
import frc.robot.commands.PidNeoCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.RandomNeo;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final RandomNeo randomNeo = new RandomNeo();
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
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, 2).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoystick, 3).whileTrue(new PidNeoCmd(randomNeo, 0));
        new JoystickButton(driverJoystick, 4).whileTrue(new PidNeoCmd(randomNeo, 10));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
