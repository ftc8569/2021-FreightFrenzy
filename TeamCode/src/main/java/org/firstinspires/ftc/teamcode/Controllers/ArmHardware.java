package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class ArmHardware {

    public abstract void init(HardwareMap hw);

    public abstract void set(Double power);
}
