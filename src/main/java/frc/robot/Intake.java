package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake {
  private DoubleSolenoid solenoid;
  private CANSparkMax primaryMotor;
  private CANSparkMax secondaryMotor;

  public Intake() {
    primaryMotor = Constants.primaryIntakeMotor.build();
    secondaryMotor = Constants.secondaryIntakeMotor.build();
    solenoid = Constants.intakeSolenoid.build();
  }

  public void toggleIntake() {
    if (solenoid.get() == DoubleSolenoid.Value.kForward) {
      retractIntake();
      spinIntake(Constants.intakeSpinVelocity);
    } else {
      extendIntake();
      spinIntake(0.0d);
    }
  }

  public void extendIntake() {
    if (solenoid.get() != DoubleSolenoid.Value.kForward) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retractIntake() {
    if (solenoid.get() != DoubleSolenoid.Value.kReverse) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void spinIntake(double vel) {
    primaryMotor.set(vel);
  }
}
