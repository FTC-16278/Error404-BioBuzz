package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the Digital Sensor class, which constructs and configures the sensor hardware.
 */
public class DigitalSensor {
    private DigitalChannel sensor;
    private DigitalChannel.Mode mode = DigitalChannel.Mode.INPUT;
    private String sensorName = "digitalSensor";

    public DigitalSensor sensorName(String sensorName) {
        this.sensorName = sensorName;
        return this;
    }

    public DigitalSensor sensorMode(DigitalChannel.Mode mode) {
        this.mode = mode;
        return this;
    }

    public void build(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(DigitalChannel.class, sensorName);
        sensor.setMode(mode);
    }

    /**
     * For the input mode, this method outputs true/false when the sensor hardware detects any
     * object in a valid distance.
     */
    public boolean isDetected() {
        return sensor.getState();
    }
}
