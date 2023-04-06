package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
	enum SpinState {
		None,
		Outake,
		Intake
	}

	private DoubleSolenoid solenoid;
	private CANSparkMax intakeMotor;
	private SpinState spinState;

	public Claw() {
		solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 7);
		intakeMotor = new CANSparkMax(14, MotorType.kBrushed);
		intakeMotor.setIdleMode(IdleMode.kBrake);
		// intakeMotor.setSmartCurrentLimit(20);
		this.spinState = SpinState.None;

		closeClaw();
	}

	public void onRobotEnable() {
		if (solenoid.get() == DoubleSolenoid.Value.kOff) {
			solenoid.set(DoubleSolenoid.Value.kForward);
		}
	}

	public boolean isClosed() {
		return solenoid.get() == DoubleSolenoid.Value.kForward;
	}

	public void toggleClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			openClaw();
		} else {
			closeClaw();
		}
	}

	public void closeClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kForward) {
			return;
		}
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void openClaw() {
		if (solenoid.get() == DoubleSolenoid.Value.kReverse) {
			return;
		}
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void spinIntake(boolean inwards) {
		if (inwards) {
			spinState = SpinState.Intake;
			intakeMotor.set(-Constants.intakePercentVel);
		} else {
			spinState = SpinState.Outake;
			intakeMotor.set(Constants.intakePercentVel);
		}
	}

	public void stopIntake() {
		intakeMotor.set(0);
		spinState = SpinState.None;
	}

	public boolean isOutaking() {
		return spinState == SpinState.Outake;
	}

	public boolean isIntaking() {
		return spinState == SpinState.Intake;
	}

	public boolean isStopped() {
		return spinState == SpinState.None;
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Claw Current (Amps)",
		// intakeMotor.getOutputCurrent());

		SmartDashboard.putString("claw state: ", solenoid.get().name());
	}
}
