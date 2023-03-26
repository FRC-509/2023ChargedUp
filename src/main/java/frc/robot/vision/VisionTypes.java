package frc.robot.vision;

public class VisionTypes {
	public enum CameraType {
		FRONT,
		BACK,
		NONE
	}

	public enum TargetType {
		SUBSTATION,
		CUBE,
		CONE,
		NONE
	}

	public enum PipelineState {
		AprilTags(0),
		RetroReflective(1),
		MLGamePieces(2);

		final int value;

		PipelineState(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum GamePiece {
		Cone(0),
		Cube(1),
		Merge(2);

		final int value;

		GamePiece(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}
}
