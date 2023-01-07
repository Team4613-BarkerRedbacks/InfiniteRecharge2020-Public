package redbacks.robot.subsystems.indexer;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import arachne.lib.pipeline.BooleanSink;

public class TestIndexer {
	Indexer indexer;

	@BeforeEach
	void beforeEach() {
		indexer = new Indexer();
		indexer.initialize();
	}

	@Test
	void shooterAtTargetSpeed() {
		BooleanSink shooterAtTargetSpeed = new BooleanSink(false);
		indexer.getShooterAtTargetSpeedInput().populate(shooterAtTargetSpeed);

		indexer.shootInAuto();
		indexer.run();
		assertEquals(0, indexer.getPowerOutput().get());

		shooterAtTargetSpeed.accept(true);
		indexer.run();
		assertEquals(1, indexer.getPowerOutput().get());

		shooterAtTargetSpeed.accept(false);
		indexer.run();
		assertEquals(0, indexer.getPowerOutput().get());

		indexer.index();
		indexer.run();
		assertEquals(0, indexer.getPowerOutput().get());
	}
}
