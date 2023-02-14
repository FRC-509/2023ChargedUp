package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private DoubleSolenoid solenoid;
  private CANSparkMax primaryMotor;
  private CANSparkMax secondaryMotor;

  public Intake() {
    primaryMotor = new CANSparkMax(9, MotorType.kBrushless);
    secondaryMotor = new CANSparkMax(12, MotorType.kBrushless);
    // secondaryMotor.setInverted(true);
    // secondaryMotor.follow(primaryMotor);
    solenoid = new DoubleSolenoid(18, PneumaticsModuleType.REVPH, 5, 7);
  }

  public void drop() {
    if (solenoid.get() != DoubleSolenoid.Value.kForward) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retract() {
    if (solenoid.get() != DoubleSolenoid.Value.kReverse) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void spin(double v) {
    primaryMotor.set(v);
    // secondaryMotor.set(-v);
  }
}
