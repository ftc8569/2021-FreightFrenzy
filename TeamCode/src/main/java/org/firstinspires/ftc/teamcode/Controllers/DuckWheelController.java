package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DuckWheelController {
    public DcMotorEx duckwheel1, duckwheel2;

    public static double defaultAccelCoeff = 2, defaultMinSpeed = 0, defaultMaxSpeed = .95, defaultTime = 2, defaultExp = 1.5;
    public double accelCoeff = 2, minSpeed = 0, maxSpeedFast = .95, maxSpeedSlow = .8, time = 2, exp = 1.5;

    public double currentPower = 0;

    private double maxSpeed = maxSpeedFast;

    public ElapsedTime timer = new ElapsedTime(0);

    boolean reversed = false;

    public DuckWheelController(DcMotorEx duckwheel1, DcMotorEx duckwheel2) {
        this.duckwheel1 = duckwheel1;
        this.duckwheel2 = duckwheel2;
    }

    public void update() {
        if(timer.seconds() < time) {
            currentPower = Math.min(Math.pow(timer.seconds() * accelCoeff, exp) + minSpeed, maxSpeed);
        } else currentPower = 0;
        duckwheel2.setPower(currentPower);
        duckwheel1.setPower(currentPower);
    }

    public void spinDuck() {
        spin(0, .95, 1.75, 2.25, 1.5);

    }

    public void spinDuckSlow() {
        spin(0, .8, 2, 2, 1.5);
    }

    public void spinDuckAuto() {
        spin(0, .8, 2, 2, 1.5);
    }

    public void slingDuck() {
        spin(0, 1, 2, 5, 4);
    }

    protected void spin(double minSpeed, double maxSpeed, double time, double accelCoeff, double exp) {
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.time = time;
        this.accelCoeff = accelCoeff;
        this.exp = exp;
        timer.reset();
    }

    public void reverse() {
        reversed = !reversed;
        duckwheel1.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        duckwheel2.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void setReverse(boolean reverse) {
        reversed = reverse;
        duckwheel1.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        duckwheel2.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public boolean isBusy() {
        return currentPower != 0;
    }


}
