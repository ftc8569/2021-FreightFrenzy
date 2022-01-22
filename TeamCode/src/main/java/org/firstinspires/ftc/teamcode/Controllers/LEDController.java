package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDController {
    RevBlinkinLedDriver led;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    ElapsedTime ledTimer;

    public LEDController(RevBlinkinLedDriver led) {
        this.led = led;
        this.ledTimer = new ElapsedTime(0);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(ledTimer.seconds() > 1 && this.pattern != pattern) {
            led.setPattern(pattern);
            ledTimer.reset();
            this.pattern = pattern;
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }

    public double getSeconds() {
        return ledTimer.seconds();
    }
}
