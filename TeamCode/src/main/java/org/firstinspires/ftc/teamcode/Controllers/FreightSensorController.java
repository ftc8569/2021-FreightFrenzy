package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class FreightSensorController {
    private RevColorSensorV3 sensor;
    private AnalogInput metalDetector;

    public static double FREIGHT_THRESHOLD = 570;
    public static double CUBE_THRESHOLD = 400;
    public double red = 0, blue = 0, green = 0, alpha = 0;

    public boolean hasMetal = false;

    public boolean hasFreight = false, lastHadFreight = false;

    public Freight freight = Freight.NONE;

    public enum Freight {
        NONE,
        BALL,
        CUBE,
        HEAVYCUBE
    }

    public FreightSensorController(RevColorSensorV3 sensor, AnalogInput metalDetector) {
        this.sensor = sensor;
        this.metalDetector = metalDetector;
    }

    public void update() {
        hasMetal = metalDetector.getVoltage() > 1.5;


        red = sensor.red();
        green = sensor.green();
        blue = sensor.blue();
        alpha = sensor.alpha();

        hasFreight = red + green + blue + alpha > FREIGHT_THRESHOLD;

        if(hasFreight && lastHadFreight) {
            if(red < CUBE_THRESHOLD) {
                freight = Freight.BALL;
            } else freight = Freight.CUBE;
        } else freight = Freight.NONE;

        if(hasMetal) {
            hasFreight = true;
            freight = Freight.HEAVYCUBE;
        }

        lastHadFreight = hasFreight;
    }

    public double[] getRGBA() {
        return new double[]{red, green, blue, alpha};
    }

    public boolean hasFreight() {
        return hasFreight;
    }

    public Freight getFreight() {
        return freight;
    }

    public double getSum() {
        return red + green + blue + alpha;
    }
}
