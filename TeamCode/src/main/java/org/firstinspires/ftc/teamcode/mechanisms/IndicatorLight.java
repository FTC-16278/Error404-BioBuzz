package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the RGB Indicator Light (PWM controlled) class.
 */
public class IndicatorLight {
    private Servo indicatorLight;
    private String indicatorName = "indicatorLight";

    public enum IndicatorColor {
        OFF(0.0), RED(0.290), ORANGE(0.333),
        YELLOW(0.388), SAGE(0.444), GREEN(0.500),
        AZURE(0.555), BLUE(0.611), INDIGO(0.666),
        VIOLET(0.722), WHITE(1.0);

        final double indicatorPower;

        IndicatorColor(double indicatorPower) {
            this.indicatorPower = indicatorPower;
        }
    }

    public IndicatorLight indicatorName(String indicatorName) {
        this.indicatorName = indicatorName;
        return this;
    }

    public void build(HardwareMap hardwareMap) {
        indicatorLight = hardwareMap.get(Servo.class, indicatorName);
        indicatorLight.setPosition(IndicatorColor.OFF.indicatorPower);
    }

    public void setIndicatorColor(IndicatorColor color) {
        indicatorLight.setPosition(color.indicatorPower);
    }
}
