package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
	private DoubleSolenoid solenoid;
	// private CANSparkMax intakeMotor;

	public Claw() {
		solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
		// intakeMotor = new CANSparkMax(0xFF, MotorType.kBrushed);
	}

	public void toggleClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			retractClaw();
		} else {
			extendClaw();
		}
	}

	public void extendClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			return;
		}
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void retractClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kReverse) {
			return;
		}
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void spinIntake() {
		// intakeMotor.set(Constants.intakePercentVel);
	}
}
