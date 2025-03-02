package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax IntakeMotor = new SparkMax(8, MotorType.kBrushless);
    private final DigitalInput CoralSensor = new DigitalInput(0);

    

    public IntakeSubsystem() {
        // Initialize motor if needed, like setting idle mode
    }

    // Always run intake motor to collect coral until coral is detected
    public void collectCoral() {
        if (!CoralSensor.get()) {
            IntakeMotor.set(-0.08889);
        } else {
            stopIntake();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor", CoralSensor.get());
  
    }

    // Run intake motor backward to collect algae / outtake coral
    public void collectAlgaeOrOuttakeCoral() {
        IntakeMotor.set(-0.08889);
    }

    // Run intake motor forward to outtake algae
    public void outtakeAlgae() {
        IntakeMotor.set(0.08889);
    }

    // Stop intake motor
    public void stopIntake() {
        IntakeMotor.set(0.0);
    }

}
