package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdoRetractionController {
    protected Servo odometryServo, odometryServo1;

    private boolean isUp = false;

    public static double odometryDownPos = .3,
            odometryUpPos = .05;

    public OdoRetractionController(HardwareMap hardwareMap) {
        odometryServo = hardwareMap.get(Servo.class, "odometryServo");
        odometryServo.setPosition(odometryDownPos);

        odometryServo1 = hardwareMap.get(Servo.class, "odometryServo1");
        odometryServo1.setPosition(odometryDownPos);
    }

    public void set(boolean up) {
        odometryServo.setPosition(up ? odometryUpPos : odometryDownPos);
        odometryServo1.setPosition(up ? odometryUpPos : odometryDownPos);
        isUp = up;
    }

    public boolean isUp() {
        return isUp;
    }
}
