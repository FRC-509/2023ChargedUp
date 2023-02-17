package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Claw {
  private DoubleSolenoid solenoid;

  public Claw() {
    solenoid = Constants.clawSolenoid.build();
  }

  public void toggleClaw() {
    if (solenoid.get() == DoubleSolenoid.Value.kForward) {
      retractClaw();
    } else {
      extendClaw();
    }
  }

  public void extendClaw() {
    if (solenoid.get() != DoubleSolenoid.Value.kForward) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retractClaw() {
    if (solenoid.get() != DoubleSolenoid.Value.kReverse) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
