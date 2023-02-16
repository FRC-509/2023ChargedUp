package frc.robot.subsystems;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class Led {
  public final AddressableLED led;

  public Led(){
    this.led = new AddressableLED(0);
  }

  public void changeColor(int r, int g, int b) {
    var buffer = new AddressableLEDBuffer(Constants.ledPixelCount);
    buffer.setRGB(0, r, g, b);
    this.led.setData(buffer);
  }
}
