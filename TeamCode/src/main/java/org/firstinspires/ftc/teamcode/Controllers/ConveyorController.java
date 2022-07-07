package org.firstinspires.ftc.teamcode.Controllers;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
public class ConveyorController {

    public static double spinTime = 2;
    public static double conveyorPower = 1;

    public DcMotorEx conveyorMotor;

    public ElapsedTime timer = new ElapsedTime(0);

    int direction = 1;

    public ConveyorController(DcMotorEx conveyorMotor) {
        this.conveyorMotor = conveyorMotor;
    }

    public void update() {
        if(timer.seconds() < spinTime) {
            conveyorMotor.setPower(conveyorPower * direction);
        } else conveyorMotor.setPower(0);
    }
    public void spin(boolean reversed) {
        timer.reset();

        direction = reversed ? -1 : 1;
    }

    public boolean isBusy() {
        return timer.seconds() < spinTime;
    }
}
