package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmController {
    private HardwareMap hardwareMap;
    private ArmHardware hardware;

    public ArmController(Class<? extends ArmHardware> hardware, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        try {
            this.hardware = hardware.newInstance();
            this.hardware.init(hardwareMap);
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public void setPower(double power) {
        hardware.set(power);
    }
}
