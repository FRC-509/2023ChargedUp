package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private DoubleSolenoid solenoid;

  public Claw() {
    solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
  }

  public void openClose() {
    if (solenoid.get() == DoubleSolenoid.Value.kOff) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      solenoid.toggle();
    }
  }

  public void open() {
    if (solenoid.get() != DoubleSolenoid.Value.kForward) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void close() {
    if (solenoid.get() != DoubleSolenoid.Value.kReverse) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
