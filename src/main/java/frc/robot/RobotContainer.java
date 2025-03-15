// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AlignToReefCmd;
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
    private final Joystick driverJoystick = new Joystick(OperatorConstants.kDriverControllerPort);
    private final ManuipulatorCommand manipulatorCommand = new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem);



    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OperatorConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OperatorConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OperatorConstants.kDriverRotationAxis),
            () -> !driverJoystick.getRawButton(OperatorConstants.kDriverFieldOrientedButtonId)
        ));

        configureBindings();
        elevatorSubsystem.setDefaultCommand(new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem));
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> swerveSubsystem.resetEncoders()));
        new JoystickButton(driverJoystick, 1).whileTrue(new AlignToReefCmd(swerveSubsystem));


        //new JoystickButton(driverJoystick, 1).whileTrue(new InstantCommand(() -> elevatorSubsystem.increaseElevatorStage(), elevatorSubsystem));
        //new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> intakeSubsystem.collectAlgaeOrOuttakeCoral(), intakeSubsystem));
        //new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> intakeSubsystem.outtakeAlgae(), intakeSubsystem));

    }

    public Command getAutonomousCommand() {
// 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0).fromDegrees(0)),
            List.of(
                new Translation2d(1.2, 0)),
                new Pose2d(2.4,-0.1, new Rotation2d().fromDegrees(-115)),
            trajectoryConfig );
        
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

        // 5. Add some "wtf is this guy saying" and return everything
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new AlignToReefCmd(swerveSubsystem),
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
        

    }
}
