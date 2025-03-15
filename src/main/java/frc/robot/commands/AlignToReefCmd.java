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

  public AlignToReefCmd(SwerveSubsystem swerveSubsystem) {
    xController = new PIDController(1, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(0.06, 0, 0);  // Rotation
    xController.setTolerance(1);
    yController.setTolerance(1);
    rotController.setTolerance(1);
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2] + 1);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);
      if (!xController.atSetpoint() && !yController.atSetpoint() && !rotController.atSetpoint()) {
        swerveSubsystem.drive(xSpeed, ySpeed, rotValue);
      } else {
        swerveSubsystem.drive(0, 0, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}