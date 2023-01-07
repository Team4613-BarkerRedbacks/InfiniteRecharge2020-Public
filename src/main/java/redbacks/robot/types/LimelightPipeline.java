package redbacks.robot.types;

import redbacks.lib.hardware.Limelight;

public enum LimelightPipeline implements Limelight.Pipeline {
	ODOMETRY_3D(0);

	private final int pipelineIndex;

	private LimelightPipeline(int pipelineIndex) {
		this.pipelineIndex = pipelineIndex;
	}

	@Override
	public int getIndex() {
		return pipelineIndex;
	}
}
