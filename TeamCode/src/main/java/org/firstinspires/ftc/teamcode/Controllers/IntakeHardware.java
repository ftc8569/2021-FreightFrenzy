package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A class to be extended and the methods to be implemented to fit your specific hardware and the
 * way it needs to be initialized and power set to it.
 * @see IntakeController
 */
public abstract class IntakeHardware {

    /**
     * This needs to be implemented to correctly initialize your hardware. If this is not done
     * correctly then IntakeController cannot correctly control it.
     */
    public abstract void init(HardwareMap hw);

    /**
     * This seems like a very useless method but depending on your intake set up you might want to
     * use a unique way of setting powers and making your intake move. It also allows a simple way
     * for any number of motors (or using servos) to be allowed.
     * @param power the power to set your intake to. Positive should be in and negative out.
     *              Make sure that power 0 stops your motor.
     */
    public abstract void set(Double power);
}
