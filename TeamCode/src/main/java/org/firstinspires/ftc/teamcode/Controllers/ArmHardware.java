package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public abstract class ArmHardware {

    public abstract void init(HardwareMap hw, DcMotor.RunMode mode);

    public abstract void set(Double power);

    public abstract double getPosition();

    public abstract void setMode(DcMotor.RunMode mode);

    public abstract void setPosition(int position);

    public abstract PIDFCoefficients getPID();

    public abstract PIDFCoefficients getPositionPID();

    public abstract void setPIDF(PIDFCoefficients pidf, DcMotor.RunMode runMode);
}
