package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final Joystick driverJoystick = new Joystick(0);
    private boolean StageUp = false;
    private boolean StageDown = false;
    private boolean GamePieceTogglePressed = false;

    private final ElevatorSubsystem elevatorSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    public boolean isGamePieceCoral = true; // true for coral, false for algae
    public boolean isClawOut = false; // true for out, false for in (Not functional or needed at the moment, should get rid of it for clarity)
    public int coralLevel; // 5 Coral levels, intake, L1, L2, L3, and L4.
    public int algaeLevel; // 4 Algae levels, processer, low, high, and barge.

    public ManuipulatorCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(elevatorSubsystem, clawSubsystem, intakeSubsystem);

    }

    @Override
    public void initialize() {
        System.out.println("ManipulatorCommand initialized");
        isClawOut = false;
    }

    @Override
    public void execute() {
        //ignore unless teleop...
        if (DriverStation.isTeleop()) {  //This has to be done because the command to move the elevator in auto actually has its own variables despite them only being declared once for some reason.
            UpdateClawPosition();        //They conflict and make it jitter around

            UpdateElevatorPositionIfSafe();
        }
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
            JoystickButton ForceResetElevatorButton = new JoystickButton(driverJoystick, 4);
            if (ForceResetElevatorButton.getAsBoolean()) {
                clawSubsystem.ForceResetEncoders();
                coralLevel = 0;
                algaeLevel = 0;
                if(!isGamePieceCoral) {
                    SwapGamePiece();
                }
                UpdateElevatorPosition();
                UpdateClawPosition();
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
        
        // Check if the coral sensor detects that it has picked up a coral
        if (intakeSubsystem.CoralSensor.get()) {
        }
    }

    public void OuttakeGamepiece() {
        intakeSubsystem.UpdateIntakeOutput();
        // Check if the coral sensor detects that it has dropped a coral
        
        if (!intakeSubsystem.CoralSensor.get()) {
        }
    }

    public Command IntakeGamepieceCommand() {
        return new FunctionalCommand(
            // Initialize
            () -> {
                // Optional initialization logic if needed
            },
            // Execute
            () -> {
                intakeSubsystem.UpdateIntakeInput();
            },
            // End
            (interrupted) -> {
                // Optional cleanup logic if needed
            },
            // IsFinished
            () -> intakeSubsystem.IsHoldingGamepiece() // Ends when gamepiece is detected
        );
    }
    public Command OuttakeGamepieceCommand() {
        return new FunctionalCommand(
            // Initialize
            () -> {
                // Optional initialization logic if needed
            },
            // Execute
            () -> {
                intakeSubsystem.UpdateIntakeOutput();
            },
            // End
            (interrupted) -> {
                intakeSubsystem.UpdateIntakeInput();
                // Optional cleanup logic if needed
            },
            // IsFinished
            () -> !intakeSubsystem.IsHoldingGamepiece() // Ends when gamepiece drop is detected
        );
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

    /** Sets the stage of the elevator, has input for gamepiece mode as well. */
    public Command SetStage(int stageNumber, boolean gamePieceCoral) {  //Cannot be run in teleop or will conflict due to isolated variable instances with the same name??? idk
    return new FunctionalCommand(
        // Initialize
        () -> {
            if (gamePieceCoral != isGamePieceCoral) {
                SwapGamePiece();
            }
            
            if (gamePieceCoral) {
                coralLevel = stageNumber;
            } else {
                algaeLevel = stageNumber;
            }
            UpdateClawPosition();
            UpdateElevatorPositionIfSafe();
            },
        // Execute
        () -> {
            SmartDashboard.putNumber("Elevator level", coralLevel + algaeLevel);
            UpdateClawPosition();
            UpdateElevatorPositionIfSafe();
            },
        // End
        (interrupted) -> {
            // Optional cleanup logic if needed
            ForceUpdateClawPosition();  // The claw will be in its "target position" as in the safe position instead of the real target so one more run will set to actual target
        },
        // IsFinished
        () -> elevatorSubsystem.AtTarget() && clawSubsystem.AtTarget() // Ends when the elevator and claw are at their target positions
    );
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
            //UpdateElevatorPosition();
        }
    }

    public void UpdateElevatorPosition() {
        System.out.print(coralLevel);
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

    public void ForceUpdateClawPosition() {
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


}
