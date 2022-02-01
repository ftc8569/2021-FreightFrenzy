package org.firstinspires.ftc.teamcode.Development;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class VirtualAuto extends MainAutoV1
{
    @Override
    public void init_loop() {


        PoseStorage.startingPosition = PoseStorage.StartingPosition.WAREHOUSE;
        PoseStorage.alliance = PoseStorage.Alliance.RED;
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);

        position = pipeline.positionDetected;

        telemetry.addData("looping", "...");
        telemetry.addData("Position", position.toString());
        telemetry.update();
    }
}
