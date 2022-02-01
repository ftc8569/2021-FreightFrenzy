package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DuckWheelController {
    public DcMotorEx duckwheel1, duckwheel2;

    public static double accelCoeff = 3, minSpeed = 0, maxSpeed = .95, time = 1.75, exp = 1.5;

    public double currentPower = 0;

    public ElapsedTime timer = new ElapsedTime( );

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
        timer.reset();
    }

    public void reverse() {
        reversed = !reversed;
        duckwheel1.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        duckwheel2.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
}
