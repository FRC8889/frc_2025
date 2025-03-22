// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
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

        
        elevatorSubsystem.setDefaultCommand(new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem));
        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoystick, 3).whileTrue(new AlignToReefCmd(swerveSubsystem, "left"));
        new JoystickButton(driverJoystick, 2).whileTrue(new AlignToReefCmd(swerveSubsystem, "right"));
        new JoystickButton(driverJoystick, 1).whileTrue(new AlignToReefCmd(swerveSubsystem, "middle"));



        //new JoystickButton(driverJoystick, 1).whileTrue(new InstantCommand(() -> elevatorSubsystem.increaseElevatorStage(), elevatorSubsystem));
        //new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> intakeSubsystem.collectAlgaeOrOuttakeCoral(), intakeSubsystem));
        //new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> intakeSubsystem.outtakeAlgae(), intakeSubsystem));

    }

    public int extractReefNumber(String input) {
        // Define the regex pattern to match "reef" followed by a number
        Pattern pattern = Pattern.compile("reef(\\d+)");
        Matcher matcher = pattern.matcher(input);
        System.out.println("5 generation");
        // Check if the pattern matches
        if (matcher.find()) {
            // Extract the number as a string and convert it to an integer
            System.out.println("6 generation");
            return Integer.parseInt(matcher.group(1));
        } else {
            // Return a default value or throw an exception if no match is found
            return 3;
        }
        
    }
    
    public int extractLevelNumber(String input) {
        // Define the regex pattern to match "l" followed by a number
        Pattern pattern = Pattern.compile("l(\\d+)");
        Matcher matcher = pattern.matcher(input);
    
        // Check if the pattern matches
        if (matcher.find()) {
            // Extract the number as a string and convert it to an integer
            return Integer.parseInt(matcher.group(1));
        } else {
            // Return a default value or throw an exception if no match is found
            return 3;
        }
    }













    public Command getAutonomousCommand() {
        System.out.println("1 generation");
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
        System.out.println("2 generation");

        double startPositionY = SmartDashboard.getNumber("% Along Line", 0);
        startPositionY = 0.5 + (startPositionY / 100.0) * (AutoConstants.kLineLeftY - AutoConstants.kLineRightY); // Interpolate
        String autoPlacement1 = SmartDashboard.getString("Auto placement 1", "reef2 l3 right").toLowerCase();
        String autoPlacement2 = SmartDashboard.getString("Auto placement 2", "reef2 l3 left").toLowerCase();
        String autoPlacement3 = SmartDashboard.getString("Auto placement 3", "reef3 l3 left").toLowerCase();
        String gamepieceSource = SmartDashboard.getString("Gamepiece Source", "right").toLowerCase();
        double intakeXPosition;
        double intakeYPosition;
        double intakeTheta;
        System.out.println("3 generation");

        if (gamepieceSource.contains("l")) {
            intakeXPosition = AutoConstants.CoralIntakePositionsX[1];
            intakeYPosition = AutoConstants.CoralIntakePositionsY[1];
            intakeTheta = AutoConstants.CoralIntakeThetas[1];
        } else {
            intakeXPosition = AutoConstants.CoralIntakePositionsX[2];
            intakeYPosition = AutoConstants.CoralIntakePositionsY[2];
            intakeTheta = AutoConstants.CoralIntakeThetas[2];
        }
        System.out.println("4 generation");


        // 2. Generate trajectories
        Trajectory drive2ScorePos1Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kLineX, startPositionY, new Rotation2d(0).fromDegrees(180)),
            List.of(
                new Translation2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement1)], startPositionY)),
                new Pose2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement1)], AutoConstants.kReefYPositions[extractReefNumber(autoPlacement1)], new Rotation2d().fromDegrees(-AutoConstants.kReefThetas[extractReefNumber(autoPlacement1)])),
            trajectoryConfig );

        Trajectory drive2IntakePos1Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement1)], AutoConstants.kReefYPositions[extractReefNumber(autoPlacement1)], new Rotation2d(0).fromDegrees(AutoConstants.kReefThetas[extractReefNumber(autoPlacement1)])),
            List.of(
                new Translation2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement1)], intakeYPosition)),
                new Pose2d(intakeXPosition, intakeYPosition, new Rotation2d().fromDegrees(intakeTheta)),
            trajectoryConfig );

        Trajectory drive2ScorePos2Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(intakeXPosition, intakeYPosition, new Rotation2d().fromDegrees(intakeTheta)),
            List.of(
                new Translation2d(intakeXPosition, AutoConstants.kReefYPositions[extractReefNumber(autoPlacement2)])),
                new Pose2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement2)], AutoConstants.kReefYPositions[extractReefNumber(autoPlacement2)], new Rotation2d().fromDegrees(AutoConstants.kReefThetas[extractReefNumber(autoPlacement2)])),
            trajectoryConfig );

        Trajectory drive2IntakePos2Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement2)], AutoConstants.kReefYPositions[extractReefNumber(autoPlacement2)], new Rotation2d().fromDegrees(AutoConstants.kReefThetas[extractReefNumber(autoPlacement2)])),
            List.of(
                new Translation2d(intakeXPosition, AutoConstants.kReefYPositions[extractReefNumber(autoPlacement2)])),
                new Pose2d(intakeXPosition, intakeYPosition, new Rotation2d().fromDegrees(intakeTheta)),
            trajectoryConfig );

        Trajectory drive2ScorePos3Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(intakeXPosition, intakeYPosition, new Rotation2d().fromDegrees(intakeTheta)),
            List.of(
                new Translation2d(intakeXPosition, AutoConstants.kReefYPositions[extractReefNumber(autoPlacement3)])),
                new Pose2d(AutoConstants.kReefXPositions[extractReefNumber(autoPlacement3)], AutoConstants.kReefYPositions[extractReefNumber(autoPlacement3)], new Rotation2d().fromDegrees(AutoConstants.kReefThetas[extractReefNumber(autoPlacement3)])),
            trajectoryConfig );

        
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //4. Construct command to follow trajectory
        SwerveControllerCommand drive2ScorePos1 = new SwerveControllerCommand(drive2ScorePos1Trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
        SwerveControllerCommand drive2IntakePos1 = new SwerveControllerCommand(drive2IntakePos1Trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
        SwerveControllerCommand drive2ScorePos2 = new SwerveControllerCommand(drive2ScorePos2Trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
        SwerveControllerCommand drive2IntakePos2 = new SwerveControllerCommand(drive2IntakePos2Trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
        SwerveControllerCommand drive2ScorePos3 = new SwerveControllerCommand(drive2ScorePos3Trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

        // 5. Add some "wtf is this guy saying" and return everything
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(drive2ScorePos1Trajectory.getInitialPose())),
            
            drive2ScorePos1,
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(extractLevelNumber(autoPlacement1), true),
            new AlignToReefCmd(swerveSubsystem, SmartDashboard.getString("Auto placement 1", "reef2 l3 right")),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).OuttakeGamepieceCommand(),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(0, true),
            
            drive2IntakePos1,
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).IntakeGamepieceCommand(),
            
            drive2ScorePos2,
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(extractLevelNumber(autoPlacement2), true),
            new AlignToReefCmd(swerveSubsystem, SmartDashboard.getString("Auto placement 2", "reef2 l3 left")),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).OuttakeGamepieceCommand(),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(0, true),

            drive2IntakePos2,
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).IntakeGamepieceCommand(),

            drive2ScorePos3,
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(extractLevelNumber(autoPlacement3), true),
            new AlignToReefCmd(swerveSubsystem, SmartDashboard.getString("Auto placement 3", "reef3 l3 right")),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).OuttakeGamepieceCommand(),
            new ManuipulatorCommand(elevatorSubsystem, clawSubsystem, intakeSubsystem).SetStage(0, true),

            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
        

    }
}
