package org.firstinspires.ftc.teamcode.Controllers;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

//@Config
public class DepositController {
    public static double kickerStartPos = 0,
            kickerHoldPos = .12,
            kickerKickPos = .475,
            armServoShutPos = 0.09,
            armServoOpenPos = .2;
    Servo doorServo, kickerServo;

    boolean out;

    public DepositController(Servo doorServo, Servo kickerServo) {
        this.doorServo = doorServo;
        this.kickerServo = kickerServo;
    }

    public void set(boolean out) {
        doorServo.setPosition(out ? armServoOpenPos : armServoShutPos);
        kickerServo.setPosition(out ? kickerKickPos : kickerStartPos);

        this.out = out;
    }

    public void hold() {
        kickerServo.setPosition(kickerHoldPos);
    }

    public boolean getOut() {
        return out;
    }

}
