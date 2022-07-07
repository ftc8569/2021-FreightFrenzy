package org.firstinspires.ftc.teamcode.Controllers;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//@Config
public class ArmController {
    private HardwareMap hardwareMap;
    private ArmHardware hardware;

    public static double armSetPosPower = .9; //tuen this down if too fast when going to positions

    protected double setPos = 0;

    public ArmController(Class<? extends ArmHardware> hardware, HardwareMap hardwareMap, DcMotor.RunMode mode) {
        this.hardwareMap = hardwareMap;
        try {
            this.hardware = hardware.newInstance();
            this.hardware.init(hardwareMap, mode);
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public void setPower(double power) {
        hardware.set(power);
    }

    public double getPosition() {
        return hardware.getPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        hardware.set(0.0); //safety reasons
        hardware.setMode(mode);
        if(mode != DcMotor.RunMode.RUN_TO_POSITION) {
            setPos = -1;
        }
    }

    //this could have some unwanted behavior if used while not in RUN_TO_POSITION so it might should be redone slightly
    public void setPosition(int position) {
        hardware.set(armSetPosPower);
        hardware.setPosition(position);
        setPos = position;
    }

    public double getSetPosition() {
        return setPos;
    }

    public PIDFCoefficients getPID() {
        return hardware.getPID();
    }

    public PIDFCoefficients getPositionPID() {
        return hardware.getPositionPID();
    }

    public void setPIDF(PIDFCoefficients pidf, DcMotor.RunMode runMode) {
        hardware.setPIDF(pidf, runMode);
    }
}
