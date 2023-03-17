package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StatefulSubsystem<T extends Comparable<T>> extends SubsystemBase {
	T currentState;

	public StatefulSubsystem(T startingState) {
		this.currentState = startingState;
	}

	public void manageState() {
		if (DriverStation.isAutonomous()) {
			return;
		}

		T newState = setState();

		if (newState != currentState) {
			onStateExit(currentState);
			onStateEnter(newState);
			currentState = newState;
		}

		onStateUpdate(currentState);
	}

	@Override
	public void periodic() {
		manageState();
		super.periodic();
	}

	abstract public T setState();

	abstract public void onStateEnter(T state);

	abstract public void onStateExit(T state);

	abstract public void onStateUpdate(T state);
}
