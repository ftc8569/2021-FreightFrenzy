package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class FreightSensorController {
    private RevColorSensorV3 sensor;

    public static double FREIGHT_THRESHOLD = 570;
    public static double CUBE_THRESHOLD = 400;
    public double red = 0, blue = 0, green = 0, alpha = 0;

    public enum Freight {
        NONE,
        BALL,
        CUBE
    }

    public FreightSensorController(RevColorSensorV3 sensor) {
        this.sensor = sensor;
    }

    public void update() {
        red = sensor.red();
        green = sensor.green();
        blue = sensor.blue();
        alpha = sensor.alpha();
    }

    public double[] getRGBA() {
        return new double[]{red, green, blue, alpha};
    }

    public boolean hasFreight() {
        return red + green + blue + alpha > FREIGHT_THRESHOLD;
    }

    public Freight getFreight() {
        if(red + green + blue + alpha > FREIGHT_THRESHOLD) {
            if(red < CUBE_THRESHOLD) {
                return Freight.BALL;
            } else return Freight.CUBE;
        } else return Freight.NONE;
    }

    public double getSum() {
        return red + green + blue + alpha;
    }
}
