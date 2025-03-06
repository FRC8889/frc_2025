package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ManuipulatorCommand extends Command {
    private final Joystick operatorJoystick = new Joystick(1);
    private boolean CycleStageTogglePressed = false;
    private boolean ClawPositionTogglePressed = false;
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
    }

    @Override
    public void execute() {
        // Run the stuffs to make it move
        UpdateElevatorPositionIfSafe();
        UpdateClawPosition();
        clawSubsystem.RunClawPID();
        elevatorSubsystem.RunElevatorPID();
        intakeSubsystem.UpdateIntakeInput();

        SmartDashboard.putNumber("Coral Level, Algae Level", coralLevel + algaeLevel);
        SmartDashboard.putBoolean("Claw Position", isClawOut);
        SmartDashboard.putBoolean("Piece Type", isGamePieceCoral);


        // Run Intake/Outtake always depending on press
        JoystickButton OuttakeButton = new JoystickButton(operatorJoystick, 4);
        if (OuttakeButton.getAsBoolean()) {
            OuttakeGamepiece();
        } else{
            IntakeGamepiece();
        }

        // Run CycleStage ONCE per click
        JoystickButton CycleStage = new JoystickButton(operatorJoystick, 1);
        if (CycleStage.getAsBoolean()) {
            if (!CycleStageTogglePressed) {
                CycleStage();
                CycleStageTogglePressed = true;
            }
        } else {
            CycleStageTogglePressed = false;
        }

        // Run ClawToggle ONCE per click
        JoystickButton ClawPosition = new JoystickButton(operatorJoystick, 3);
        if (ClawPosition.getAsBoolean()) {
            if (!ClawPositionTogglePressed) {
                ToggleClawPosition();
                ClawPositionTogglePressed = true;
            }
        } else {
            ClawPositionTogglePressed = false;
        }

        // run SwapGamePiece ONCE per click
        JoystickButton GamePiece = new JoystickButton(operatorJoystick, 2);
        if(GamePiece.getAsBoolean()) {
            if (!GamePieceTogglePressed) {
                SwapGamePiece();
                GamePieceTogglePressed = true;
            }
        } else {
            GamePieceTogglePressed = false;
        }
    }
       
    
    
    /** Run this by default with a reverse polarity button thing then run the other to score when that is clicked */
    public void IntakeGamepiece() {
        intakeSubsystem.UpdateIntakeInput();
    }

    public void OuttakeGamepiece() {
        intakeSubsystem.UpdateIntakeOutput();
    }

    public void ResetStage() {
        if (isGamePieceCoral) {
            coralLevel = 0;
        } else {
            algaeLevel = 0;
        }
        ResetClawPosition();
    }

    public void CycleStage() {
        if (isGamePieceCoral) {
            coralLevel++;
            if (coralLevel > 4) {
                ResetStage();
            }
        } else {
            algaeLevel++;
            if (algaeLevel > 2) {
                ResetStage();
            }
        }
        ResetClawPosition();
        return;
    }

    public void ResetClawPosition() {
        isClawOut = false;
    }

    public void ToggleClawPosition() {
        isClawOut = !isClawOut;
        return;
    }

    public void SwapGamePiece() {
        if (isGamePieceCoral) {
            algaeLevel = 0;
        } else {
            coralLevel = 0;
        }
        SwapLocalGamePiece();
        elevatorSubsystem.SwapLocalGamePiece();
        clawSubsystem.SwapLocalGamePiece();
        intakeSubsystem.SwapLocalGamePiece();
        return;
    }
    
    public void SwapLocalGamePiece() {
        isGamePieceCoral = !isGamePieceCoral;
    }


    public void UpdateElevatorPositionIfSafe() {
        if (clawSubsystem.GetClawEncoder() > ManipulatorConstants.kClawPositionSafe1Min && clawSubsystem.GetClawEncoder() < ManipulatorConstants.kClawPositionSafe1Max) {
            if (clawSubsystem.GetClawEncoder() > ManipulatorConstants.kClawPositionSafe2Min && clawSubsystem.GetClawEncoder() < ManipulatorConstants.kClawPositionSafe2Max) {
                UpdateElevatorPosition();
            } else {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
                UpdateElevatorPosition();
            }
        } else { 
            clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
            UpdateElevatorPosition();
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
                }
            }
        }

    }

    public void UpdateClawPosition() {
        if (isGamePieceCoral) {
            if (isClawOut && !elevatorSubsystem.ElevatorInMotion()) {
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
            } else if (coralLevel == 0 && !elevatorSubsystem.ElevatorInMotion()) {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionIntake);
            } else {
                clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionFallback);
            }

        } else {
            if (!isGamePieceCoral) {
                if (algaeLevel == 0) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeProcesser);
                } else if (algaeLevel == 1) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeLow);
                } else if (algaeLevel == 2) {
                    clawSubsystem.SetClawTargetPosition(ManipulatorConstants.kClawPositionAlgaeHigh);
                }
            }
        } 
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
        return !DriverStation.isTeleop();   
     }

    public static void setDefaultCommand(ManuipulatorCommand manuipulatorCommand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
    }
}
