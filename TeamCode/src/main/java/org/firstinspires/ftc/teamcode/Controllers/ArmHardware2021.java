package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ArmHardware2021 extends ArmHardware{

    private DcMotorEx armMotor;
    public static PIDFCoefficients veloPID = new PIDFCoefficients(9, 0, 0,14 ), //we might do some custom pid here later
                             positionPID = new PIDFCoefficients(10, 0, 0, 0);

    public static int targetPosTolerance = 30;

    @Override
    public void init(HardwareMap hw, DcMotor.RunMode mode) {
        armMotor = hw.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(mode);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, positionPID);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, veloPID);
        armMotor.setTargetPositionTolerance(targetPosTolerance);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void set(Double power) {
        armMotor.setPower(power);
    }

    @Override
    public double getPosition() {
        return armMotor.getCurrentPosition();
    }

    @Override
    public void setMode(DcMotor.RunMode mode) {
        armMotor.setMode(mode);
    }

    @Override
    public void setPosition(int position) {
        armMotor.setTargetPosition(position);
    }

    @Override
    public PIDFCoefficients getPID() {
        return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public PIDFCoefficients getPositionPID() {
        return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setPIDF(PIDFCoefficients pidf, DcMotor.RunMode runMode) {
        armMotor.setPIDFCoefficients(runMode, pidf);
    }
}
