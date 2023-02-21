package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private DoubleSolenoid solenoid;

  public Claw() {
    solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
  }
  public void toggleIntake() {
      if (solenoid.get() == DoubleSolenoid.Value.kForward) {
          retractIntake();
      }
      else {
          extendIntake();
      }
  }

  public void extendIntake() {
      if (solenoid.get() == DoubleSolenoid.Value.kForward) {
          return;
      }
      solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
      if (solenoid.get() == DoubleSolenoid.Value.kReverse) {
          return;
      }
      solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
