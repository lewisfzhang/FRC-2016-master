package com.team254.frc2016.vision.test;

import com.team254.frc2016.vision.VisionUpdate;
import org.junit.Test;

import static org.junit.Assert.*;

public class TestVisionUpdate {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testSingleGood() {
        String s = "{\"capturedAgoMs\":100,\"targets\":[{\"theta\":5.3,\"distance\":15.3}]}";
        VisionUpdate v = VisionUpdate.generateFromJsonString(System.nanoTime(), s);
        assertTrue(v.isValid());
        assertEquals(100, v.getCapturedAgoMs());
        assertNotNull(v.getTargets());
        assertEquals(1, v.getTargets().size());
        assertEquals(5.3, v.getTargets().get(0).getAngle().getDegrees(), kTestEpsilon);
        assertEquals(15.3, v.getTargets().get(0).getDistance(), kTestEpsilon);
    }

    @Test
    public void testMultiGood() {
        String s = "{\"capturedAgoMs\":100,\"targets\":[{\"theta\":5.3,\"distance\":15.3},{\"theta\":1.2,\"distance\":11.2}]}";
        VisionUpdate v = VisionUpdate.generateFromJsonString(System.nanoTime(), s);
        assertTrue(v.isValid());
        assertEquals(100, v.getCapturedAgoMs());
        assertNotNull(v.getTargets());
        assertEquals(2, v.getTargets().size());
        assertEquals(5.3, v.getTargets().get(0).getAngle().getDegrees(), kTestEpsilon);
        assertEquals(15.3, v.getTargets().get(0).getDistance(), kTestEpsilon);
        assertEquals(1.2, v.getTargets().get(1).getAngle().getDegrees(), kTestEpsilon);
        assertEquals(11.2, v.getTargets().get(1).getDistance(), kTestEpsilon);
    }

    @Test
    public void testBadString() {
        String s = "{\"capturedAgoMs\":100,\"targets\":[{\"theta\":5.3,\"distance\":15.3},{\"the";
        VisionUpdate v = VisionUpdate.generateFromJsonString(System.nanoTime(), s);
        assertFalse(v.isValid());
    }

    @Test
    public void testBadDataType() {
        String s = "{\"capturedAgoMs\":100,\"targets\":[{\"theta\":\"notanumber\",\"distance\":15.3}]}";
        VisionUpdate v = VisionUpdate.generateFromJsonString(System.nanoTime(), s);
        assertFalse(v.isValid());
    }
}
