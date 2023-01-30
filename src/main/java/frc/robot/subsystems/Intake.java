// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Intake extends SubsystemBase {
// private DoubleSolenoid solenoid;
// private CANSparkMax masterMotor;
// private CANSparkMax slaveMotor;

// public Intake() {
// masterMotor = new CANSparkMax(0, MotorType.kBrushed);
// slaveMotor = new CANSparkMax(1, MotorType.kBrushed);
// slaveMotor.follow(masterMotor);
// solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 0);
// }

// public void intakeSpin(double v) {
// if (solenoid.get() != DoubleSolenoid.Value.kForward) {
// solenoid.set(DoubleSolenoid.Value.kForward);
// }
// masterMotor.set(v);
// }

// public void intakeSpit(double v) {
// if (solenoid.get() != DoubleSolenoid.Value.kForward) {
// solenoid.set(DoubleSolenoid.Value.kForward);
// }
// masterMotor.set(-v);
// }

// public void intakeUp() {
// if (solenoid.get() != DoubleSolenoid.Value.kReverse) {
// solenoid.set(DoubleSolenoid.Value.kReverse);
// }
// masterMotor.set(0);
// }

// @Override
// public void periodic() {
// }
// }
