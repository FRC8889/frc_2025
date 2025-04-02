package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToReefCmd extends Command {
  private PIDController xController, yController, rotController;
  private SwerveSubsystem swerveSubsystem;
  private String scorePosition;
  boolean atTarget = false;
  boolean endCommand = false;
  double reefXOffset = 0.275;
  double reefYOffset = 0.16;
  int invertY = 1;

  public AlignToReefCmd(SwerveSubsystem swerveSubsystem, String scoreSide) {
    xController = new PIDController(1.5, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(0.07, 0, 0);  // Rotation
    this.swerveSubsystem = swerveSubsystem;
    this.scorePosition = scoreSide;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();
    atTarget = false;
    endCommand = false;
    System.out.println("lining up fr");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

      double ySpeed;
      double xSpeed;
      // get speeds from positions the gamepiece is set to or something
      if(scorePosition.contains("right")) {
        ySpeed = yController.calculate(postions[0]);
        xSpeed = xController.calculate(postions[2] + 0.95); // do not slam the claw into the reef
        invertY = 1;
      } else if(scorePosition.contains("left")) {
        ySpeed = yController.calculate(postions[0]);
        xSpeed = xController.calculate(postions[2] + 0.95);
        invertY = -1;
      } else {
        // algae
        ySpeed = yController.calculate(postions[0]);
        xSpeed = xController.calculate(postions[2] + 0.6);
      }
      
      // get rest of speeds they should all be the same, can add them in to logic if they needs changing.
      double rotValue = rotController.calculate(postions[4]);
      //set speeds
      swerveSubsystem.drive(xSpeed, ySpeed, rotValue);
      //if at target set to true to end command (workaround cause pid "atSetpoint()" is not working)
      if (xController.getError() < .05 && yController.getError() < .05 && rotController.getError() < .5) {
        atTarget = true;
      }
      if(DriverStation.isTeleop()) {
        if ((atTarget) && (invertY != 0) && (Math.abs(xSpeed) + Math.abs(ySpeed) + Math.abs(rotValue) != 0)) {
          swerveSubsystem.drive(0, 0, 0);
        
          TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 2).setKinematics(DriveConstants.kDriveKinematics);
        
          Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0).fromDegrees(0)),
            List.of(
                new Translation2d(reefXOffset/2, reefYOffset*invertY/2)),
                new Pose2d(reefXOffset, reefYOffset*invertY, new Rotation2d().fromDegrees(0)),
            trajectoryConfig );
        
          PIDController TrajectoryXController = new PIDController(0.7, 0, 0);
          PIDController TrajectoryYController = new PIDController(0.7, 0, 0);
          ProfiledPIDController TrajectoryThetaController = new ProfiledPIDController(3, 0, 0, AutoConstants.kThetaControllerConstraints);
          TrajectoryThetaController.enableContinuousInput(-Math.PI, Math.PI);

          SwerveControllerCommand drive2Reef = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, TrajectoryXController, TrajectoryYController, TrajectoryThetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

          swerveSubsystem.resetOdometry(trajectory.getInitialPose());
          drive2Reef.schedule();
          endCommand = true;
          if (drive2Reef.isFinished()) {
            endCommand = true;
          }
        }
      } else if ((atTarget) && (invertY != 0)) {
        endCommand = true;
      }
    } else {
      endCommand = true;
    }
  }

  



  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous()){
    return endCommand; 
    } else {
    return atTarget;
    }
  }
}