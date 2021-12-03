package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmHardware2021 extends ArmHardware{

    private DcMotorEx armMotor;

    @Override
    public void init(HardwareMap hw) {
        armMotor = hw.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void set(Double power) {
        armMotor.setPower(power);
    }
}
