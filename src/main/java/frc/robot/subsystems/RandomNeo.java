package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class RandomNeo extends SubsystemBase{

    private final SparkMax tinyNeo = new SparkMax(9, MotorType.kBrushless);
    private final DigitalInput sensor = new DigitalInput(0);
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Neo encoder", sensor.get() );

    }

    public double getEncoder(){
        return tinyNeo.getEncoder().getPosition();
    }

    public void setMotor(double speed){
        tinyNeo.set(speed);
    }
    
}
