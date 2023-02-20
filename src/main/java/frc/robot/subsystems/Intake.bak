package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private DoubleSolenoid solenoid;
  private CANSparkMax primaryMotor;
  private CANSparkMax secondaryMotor;

  public Intake() {
    primaryMotor = new CANSparkMax(9, MotorType.kBrushless);
    secondaryMotor = new CANSparkMax(14, MotorType.kBrushless);
    // secondaryMotor.setInverted(true);
    primaryMotor.setSmartCurrentLimit(30);
    secondaryMotor.setSmartCurrentLimit(30);
    // secondaryMotor.follow(primaryMotor);
    solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 6);
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
    primaryMotor.set(-v);
    secondaryMotor.set(v);
  }
}
