package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToReefCmd extends Command {
  private PIDController xController, yController, rotController;
  private SwerveSubsystem swerveSubsystem;
  private String scorePosition;
  boolean atTarget = false;

  public AlignToReefCmd(SwerveSubsystem swerveSubsystem, String scorePosition) {
    xController = new PIDController(1.5, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(0.07, 0, 0);  // Rotation
    this.swerveSubsystem = swerveSubsystem;
    this.scorePosition = scorePosition;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();
    atTarget = false;
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

      double ySpeed;
      double xSpeed;
      // get speeds from positions the gamepiece is set to or something
      if(scorePosition.contains("left")) {
        ySpeed = yController.calculate(postions[0] + 0.21); // coral offset 0.21
        xSpeed = xController.calculate(postions[2] + 0.95); // do not slam the claw into the reef
      } else if(scorePosition.contains("right")) {
        ySpeed = yController.calculate(postions[0] - 0.21);
        xSpeed = xController.calculate(postions[2] + 0.95);
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
      if (xController.getError() < .1 && yController.getError() < .1 && rotController.getError() < 1.5) {
        atTarget = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return atTarget;
  }
}