package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NEOSparkMax;

public class Claw extends SubsystemBase {
	private DoubleSolenoid solenoid;
	private NEOSparkMax intakeMotor;

	public Claw() {
		solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
		intakeMotor = new NEOSparkMax(0xFF);
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
		intakeMotor.set(Constants.intakePercentVel);
	}
}
