package redbacks.lib.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Represents a Limelight camera on the robot, accessed via a given NetworkTable.
 * This class is a candidate for future Arachne inclusion. Do not modify without discussion with the original author.
 * 
 * TODO Implement remaining entries
 * TODO Implement advanced usage
 * 
 * @author Sean Zammit
 */
public class Limelight<PipelineT extends Limelight.Pipeline> {
	// Read data
	private final NetworkTableEntry
			hasTarget, horizontalOffsetDegrees, verticalOffsetDegrees, targetAreaPercentage, skewDegrees, pipelineLatencyMilliseconds,
			fittedBoundingBoxShortestSideLengthPixels, fittedBoundingBoxLongestSideLengthPixels,
			roughBoundingBoxHorizontalSideLengthPixels, roughBoundingBoxVerticalSideLengthPixels,
			currentPipelineIndex, position3D, targetCornerPixelPositions;
			
	// Write data
	private final NetworkTableEntry ledMode, cameraMode, changePipelineIndex, streamMode, takeSnapshots;
	
	public Limelight(NetworkTable networkTable) {
		this.hasTarget = networkTable.getEntry("tv");
		this.horizontalOffsetDegrees = networkTable.getEntry("tx");
		this.verticalOffsetDegrees = networkTable.getEntry("ty");
		this.targetAreaPercentage = networkTable.getEntry("ta");
		this.skewDegrees = networkTable.getEntry("ts");
		this.pipelineLatencyMilliseconds = networkTable.getEntry("tl");
		this.fittedBoundingBoxShortestSideLengthPixels = networkTable.getEntry("tshort");
		this.fittedBoundingBoxLongestSideLengthPixels = networkTable.getEntry("tlong");
		this.roughBoundingBoxHorizontalSideLengthPixels = networkTable.getEntry("thor");
		this.roughBoundingBoxVerticalSideLengthPixels = networkTable.getEntry("tvert");
		this.currentPipelineIndex = networkTable.getEntry("getpipe");
		this.position3D = networkTable.getEntry("camtran");
		
		this.ledMode = networkTable.getEntry("ledMode");
		this.cameraMode = networkTable.getEntry("camMode");
		this.changePipelineIndex = networkTable.getEntry("pipeline");
		this.streamMode = networkTable.getEntry("stream");
		this.takeSnapshots = networkTable.getEntry("snapshot");
		this.targetCornerPixelPositions = networkTable.getEntry("tcornxy");
	}
	
	// Getters
	public boolean hasTarget() {
		return hasTarget.getDouble(0) == 1;
	}
	
	public double getHorizontalOffsetDegrees() {
		return horizontalOffsetDegrees.getDouble(Double.NaN);
	}
	
	public double getVerticalOffsetDegrees() {
		return verticalOffsetDegrees.getDouble(Double.NaN);
	}
	
	public double getTargetAreaPercentage() {
		return targetAreaPercentage.getDouble(Double.NaN);
	}

	public double getSkewDegrees() {
		return skewDegrees.getDouble(Double.NaN);
	}

	public double[] getTargetCornerPixelPositions() {
		return targetCornerPixelPositions.getDoubleArray(new double[0]);
	}

	// Setters
	public void setPipeline(PipelineT pipeline) {
		this.changePipelineIndex.setNumber(pipeline.getIndex());
	}
	
	public static interface Pipeline {
		int getIndex();
	}
	
	public void setLedMode(LedMode mode) {
		this.ledMode.setNumber(mode.idx);
	}
	
	public static enum LedMode {
		FROM_PIPELINE(0),
		OFF(1),
		BLINK(2),
		ON(3);

		private final int idx;
		
		private LedMode(int idx) {
			this.idx = idx;
		}
	}
}
