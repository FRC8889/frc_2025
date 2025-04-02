package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();


    public LedSubsystem() {
        // Initialize the LEDs
        ledStrip = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(180);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        LEDPattern green = LEDPattern.solid(Color.kGreen);
        green.applyTo(ledBuffer);
        ledStrip.start();

    }

    public void SetColor(int colorId) {
        if (colorId == 1) {
            LEDPattern red = LEDPattern.solid(Color.kRed);
            red.applyTo(ledBuffer);
        } else if (colorId == 2) {
            LEDPattern green = LEDPattern.solid(Color.kGreen);
            green.applyTo(ledBuffer);
        } else if (colorId == 3) {
            LEDPattern blue = LEDPattern.solid(Color.kBlue);
            blue.applyTo(ledBuffer);
        } else if (colorId == 4) {
            LEDPattern yellow = LEDPattern.solid(Color.kYellow);
            yellow.applyTo(ledBuffer);
        } else if (colorId == 5) {
            LEDPattern purple = LEDPattern.solid(Color.kPurple);
            purple.applyTo(ledBuffer);
        } else if (colorId == 6) {
            LEDPattern cyan = LEDPattern.solid(Color.kCyan);
            cyan.applyTo(ledBuffer);
        } else if (colorId == 7) {
            LEDPattern white = LEDPattern.solid(Color.kWhite);
            white.applyTo(ledBuffer);
        } else {
            LEDPattern off = LEDPattern.solid(Color.kBlack);
            off.applyTo(ledBuffer);

        }
    }

    public void UpdateLEDs() {
        if(intakeSubsystem.IsHoldingGamepiece()) {
            SetColor(4);
        } else {
            SetColor(2);
        }

        if(elevatorSubsystem.GetCollectiveElevatorCurrent() > 20) {
            SetColor(1);
        }
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        UpdateLEDs();
        System.out.println("Update Leds");
    }

}
