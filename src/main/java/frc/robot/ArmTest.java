package frc.robot;

import static frc.robot.Constants.Controller.*;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Device;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;

enum ArmState {
	PercentOutput,
	Position,
}

public class ArmTest extends StatefulSubsystem<ArmState> {
	private final LazyTalonFX leftPivotMotor;
	private final LazyTalonFX rightPivotMotor;
	private final NEOSparkMax extensionMotor;
	private final PIDController extensionPID;
	private double extensionTarget;

	public ArmTest(ArmState startingState) {
		super(startingState);

		this.leftPivotMotor = Device.Motor.leftPivot.build();
		this.rightPivotMotor = Device.Motor.rightPivot.build();
		this.extensionMotor = Device.Motor.extension.build();
		this.extensionPID = Constants.PID.extension;
		this.extensionTarget = 0.0d;
	}

	public void onStateEnter(ArmState state) {
		switch (state) {
			case PercentOutput:
				break;
			case Position:
				break;
			default:
				break;
		}
	}

	@Override
	public ArmState setState() {
		return ArmState.Position;
	}

	@Override
	public void onStateExit(ArmState state) {
		switch (state) {
			case PercentOutput:
				break;
			case Position:
				break;
			default:
				break;
		}
	}

	@Override
	public void onStateUpdate(ArmState state) {
		switch (state) {
			case PercentOutput:
				break;
			case Position:
				break;
			default:
				break;
		}
	}
}
