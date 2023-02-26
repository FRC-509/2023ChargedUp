package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake {
	private DoubleSolenoid solenoid;
	private CANSparkMax primaryMotor;
	private CANSparkMax secondaryMotor;

	public Intake() {
		primaryMotor = new CANSparkMax(9, MotorType.kBrushless);
		secondaryMotor = new CANSparkMax(14, MotorType.kBrushless);
		primaryMotor.setIdleMode(IdleMode.kCoast);
		secondaryMotor.setIdleMode(IdleMode.kCoast);
		solenoid = new DoubleSolenoid(18, PneumaticsModuleType.REVPH, 4, 6);
	}

	public void toggleIntake() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			retractIntake();
			spinIntake(0.25d);
		} else {
			extendIntake();
			spinIntake(0.0d);
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

	public void spinIntake(double velocity) {
		primaryMotor.set(velocity);
	}
}
