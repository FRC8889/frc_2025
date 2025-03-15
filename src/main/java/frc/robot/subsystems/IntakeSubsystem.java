package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax IntakeMotor = new SparkMax(8, MotorType.kBrushless);
    public final DigitalInput CoralSensor = new DigitalInput(0);
    
    private boolean isGamePieceCoral = true; // true for coral, false for algae
    private double intakeSpeedCoral = 0.3;
    private double outtakeSpeedCoral = -0.5;   
    private double intakeSpeedAlgae = 0.65;
    private double outtakeSpeedAlgae = -0.75;

    public IntakeSubsystem() {
        // Initialize motor if needed, like setting idle mode
    }

    /** Toggles game piece mode starting from Coral. */
    public void SwapLocalGamePiece() {
        isGamePieceCoral = !isGamePieceCoral;
    }
   
    /** Updates intake status depending on mode set with SwapGamePiece(). */
    public void UpdateIntakeInput() {
        if (isGamePieceCoral == true) {
            if (CoralSensor.get()) {
                IntakeMotor.set(0.0);
            } else {
                IntakeMotor.set(-intakeSpeedCoral);
            }
        } else {
            IntakeMotor.set(intakeSpeedAlgae);
        }
    }

    /** Outtakes target gamepiece set with SwapGamePiece(). */
    public void UpdateIntakeOutput() {
        if (isGamePieceCoral == true) {
            if (DriverStation.isAutonomous()) {
            IntakeMotor.set(outtakeSpeedCoral * 1.5);
            } else {
                IntakeMotor.set(outtakeSpeedCoral);
            }
        } else {
            IntakeMotor.set(outtakeSpeedAlgae);
        }
    }

    public boolean IsHoldingGamepiece() {
        if (CoralSensor.get() || IntakeMotor.getOutputCurrent() > 20) {
            return true;
        } else {
            return false;
        }     
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor", CoralSensor.get());
  
    }

    /** Stops intake motors. */
    public void StopMotors() {
        IntakeMotor.set(0);
    }

}
