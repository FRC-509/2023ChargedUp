package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
	private DoubleSolenoid solenoid;
	private CANSparkMax intakeMotor;

	public Claw() {
		solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
		intakeMotor = new CANSparkMax(14, MotorType.kBrushed);
		intakeMotor.setSmartCurrentLimit(20);
	}

	public boolean isClosed() {
		return solenoid.get() == DoubleSolenoid.Value.kForward;
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

	public void spinIntake(boolean inwards) {
		if (inwards) {
			intakeMotor.set(-Constants.intakePercentVel);
		} else {
			intakeMotor.set(Constants.intakePercentVel);
		}
	}

	public void stopIntake() {
		intakeMotor.set(0);
	}

	@Override
	public void periodic() {
		intakeMotor.set(10);
	}
}
