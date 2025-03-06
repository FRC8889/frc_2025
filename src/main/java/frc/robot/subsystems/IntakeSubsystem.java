package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax IntakeMotor = new SparkMax(8, MotorType.kBrushless);
    private final DigitalInput CoralSensor = new DigitalInput(0);
    
    private boolean isGamePieceCoral = true; // true for coral, false for algae
    private double intakeSpeedCoral = 0.3;
    private double outtakeSpeedCoral = -1;   
    private double intakeSpeedAlgae = 0.65;
    private double outtakeSpeedAlgae = -1;

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
            IntakeMotor.set(outtakeSpeedCoral);
        } else {
            IntakeMotor.set(outtakeSpeedAlgae);
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
