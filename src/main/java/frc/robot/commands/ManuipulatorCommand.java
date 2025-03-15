package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ManuipulatorCommand extends Command {
    private final Joystick operatorJoystick = new Joystick(1);
    private final Joystick driverJoystick = new Joystick(0);
    private boolean StageUp = false;
    private boolean StageDown = false;
    private boolean GamePieceTogglePressed = false;

    private final ElevatorSubsystem elevatorSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private boolean isGamePieceCoral = true; // true for coral, false for algae
    private boolean isClawOut = false; // true for out, false for in
    private int coralLevel = 0; // 5 Coral levels, intake, L1, L2, L3, and L4.
    private int algaeLevel = 0; // 3 Algae levels, processer score, low, and high.

    public ManuipulatorCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(elevatorSubsystem, clawSubsystem, intakeSubsystem);

    }

    @Override
    public void initialize() {
        System.out.println("ManipulatorCommand initialized");
        algaeLevel = 0;
        coralLevel = 0;
        isClawOut = false;
        isGamePieceCoral = true;
        ResetEncoders();
    }

    @Override
    public void execute() {
        // Run the stuffs to make it move
        UpdateClawPosition();
        UpdateElevatorPositionIfSafe();
        elevatorSubsystem.RunElevatorPID();

        SmartDashboard.putNumber("Elevator level", coralLevel + algaeLevel);
        SmartDashboard.putBoolean("Claw Position", isClawOut);
        SmartDashboard.putBoolean("Piece Type", isGamePieceCoral);

        // IGNORE controls unless teleop
        if (DriverStation.isTeleop()) {
            // Run Intake/Outtake always depending on press
            JoystickButton OuttakeButton = new JoystickButton(operatorJoystick, 1);
            if (OuttakeButton.getAsBoolean()) {
                OuttakeGamepiece();
            } else{
                IntakeGamepiece();
            }

            //Either do this or update the pid like normal
            JoystickButton ForceResetClawButton = new JoystickButton(driverJoystick, 2);
            if (ForceResetClawButton.getAsBoolean()) {
                clawSubsystem.ForceResetEncoders();
            } else {
                clawSubsystem.RunClawPID();
            }

            // Run Stage Cycles ONCE per click
            if (operatorJoystick.getRawAxis(3) > 0.15) {
                if (!StageUp) {
                    StageUp();
                    StageUp = true;
                }
            } else {
                StageUp = false;
            }

            if ((operatorJoystick.getRawAxis(2) > 0.15)) {
                if (!StageDown) {
                    StageDown();
                    StageDown = true;
                }
            } else {
                StageDown = false;
            }

            JoystickButton ResetManuipulator = new JoystickButton(operatorJoystick, 2);
            if (ResetManuipulator.getAsBoolean()) {
                ResetStage();
            }

            // run SwapGamePiece ONCE per click
            JoystickButton GamePiece = new JoystickButton(operatorJoystick, 3);
            if(GamePiece.getAsBoolean()) {
                if (!GamePieceTogglePressed) {
                    SwapGamePiece();
                    GamePieceTogglePressed = true;
                }
            } else {
                GamePieceTogglePressed = false;
            }
        }
    }

    public void ResetEncoders() {
        elevatorSubsystem.ResetEncoders();
        clawSubsystem.ResetEncoders();
    }
       
    
    
    /** Run this by default with a reverse polarity button thing then run the other to score when that is clicked */
    public void IntakeGamepiece() {
        intakeSubsystem.UpdateIntakeInput();
    }

    public void OuttakeGamepiece() {
        intakeSubsystem.UpdateIntakeOutput();
        if (!intakeSubsystem.CoralSensor.get()) {
            return;
        }
    }

    public void ResetStage() {
        if (isGamePieceCoral) {
            coralLevel = 0;
        } else {
            algaeLevel = 0;
        }
    }

    public void StageUp() {
        if (isGamePieceCoral) {
            coralLevel++;
            if (coralLevel > 4) {
                coralLevel = 4;
                return;
            }
        } else {
            algaeLevel++;
            if (algaeLevel > 3) {
                algaeLevel = 3;
                return;
            }
        }
        ResetClawPosition();
        return;
    }

    public void StageDown() {
        if (isGamePieceCoral) {
            coralLevel--;
            if (coralLevel < 0) {
                coralLevel = 0;
                return;
            }
        } else {
            algaeLevel--;
            if (algaeLevel < 0) {
                algaeLevel = 0;
                return;
            }
        }
        ResetClawPosition();
        return;
    }

    public void ResetClawPosition() {
        isClawOut = false;
    }

    
    public void SwapGamePiece() {
        SwapLocalGamePiece();
        elevatorSubsystem.SwapLocalGamePiece();
        clawSubsystem.SwapLocalGamePiece();
        intakeSubsystem.SwapLocalGamePiece();
        if (isGamePieceCoral) {
            algaeLevel = 0;
            UpdateElevatorPosition();
        } else {
            coralLevel = 0;
        }
        return;
    }
    
    public void SwapLocalGamePiece() {
        isGamePieceCoral = !isGamePieceCoral;
    }


    public void UpdateElevatorPositionIfSafe() {
        if (clawSubsystem.AtTarget() || !elevatorSubsystem.InMotion()) {
            UpdateElevatorPosition();
        } else {
            clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
        }
    }

    public void UpdateElevatorPosition() {
        if (isGamePieceCoral) {
            if (coralLevel == 0) {
                elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionIntake);
            } else if (coralLevel == 1) {
                elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionL1);
            } else if (coralLevel == 2) {
                elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionL2);
            } else if (coralLevel == 3) {
                elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionL3);
            } else if (coralLevel == 4) {
                elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionL4);
            }
        } else {
            if (!isGamePieceCoral) {
                if (algaeLevel == 0) {
                    elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionAlgaeProcesser);
                } else if (algaeLevel == 1) {
                    elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionAlgaeLow);
                } else if (algaeLevel == 2) {
                    elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionAlgaeHigh);
                }   else if (algaeLevel == 3) {
                    elevatorSubsystem.SetElevatorTargetPosition(ManipulatorConstants.kElevatorPositionAlgaeBarge);
                }
            }
        }

    }

    public void UpdateClawPosition() {
        if (isGamePieceCoral && !elevatorSubsystem.InMotion() && elevatorSubsystem.AtTarget()) {
            if (!elevatorSubsystem.InMotion()) {
                if (coralLevel == 0) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionIntake);
                } else if (coralLevel == 1) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionL1);
                } else if (coralLevel == 2) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionL2);
                } else if (coralLevel == 3) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionL3);
                } else if (coralLevel == 4) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionL4);
                }
            } else if (coralLevel == 0) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionIntake);
            } else { 
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
            }

        } else if (!isGamePieceCoral) {
            if (algaeLevel == 0) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeProcesser);
            } else if (algaeLevel == 1) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeLow);
            } else if (algaeLevel == 2) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeHigh);
            } else if (algaeLevel == 3) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeBarge);
            }
        } else {
            clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
        }
    }

    public void StopMotors() {
        elevatorSubsystem.StopMotors();
        clawSubsystem.StopMotors();
        intakeSubsystem.StopMotors();
    }

    
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.StopMotors();
        clawSubsystem.StopMotors();
        intakeSubsystem.StopMotors();
        System.out.println("ManipulatorCommand ended");
    }

    @Override
    public boolean isFinished() {
   // Only run in teleop; finish immediately otherwise.
        //return !DriverStation.isTeleop();   
        return false;
     }

    public static void setDefaultCommand(ManuipulatorCommand manuipulatorCommand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
    }
}
