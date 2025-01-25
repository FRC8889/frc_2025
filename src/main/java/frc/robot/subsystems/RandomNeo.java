package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RandomNeo extends SubsystemBase{

    private final SparkMax tinyNeo = new SparkMax(9, MotorType.kBrushless);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Neo encoder", getEncoder());

    }

    public double getEncoder(){
        return tinyNeo.getEncoder().getPosition();
    }

    public void setMotor(double speed){
        tinyNeo.set(speed);
    }
    
}
