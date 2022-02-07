package org.firstinspires.ftc.teamcode.Development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controllers.MaxBoticsArray;
import org.firstinspires.ftc.teamcode.Controllers.MaxBoticsMB1040;

import java.util.Arrays;

@Autonomous
public class InitDistanceSensors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput front = hardwareMap.get(AnalogInput.class, "frontSensor");
        MaxBoticsMB1040 frontSensor = new MaxBoticsMB1040(front);
        AnalogInput back = hardwareMap.get(AnalogInput.class, "backSensor");
        MaxBoticsMB1040 backSensor = new MaxBoticsMB1040(back);
        AnalogInput left = hardwareMap.get(AnalogInput.class, "leftSensor");
        MaxBoticsMB1040 leftSensor = new MaxBoticsMB1040(left);
        AnalogInput right = hardwareMap.get(AnalogInput.class, "rightSensor");
        MaxBoticsMB1040 rightSensor = new MaxBoticsMB1040(right);

        DigitalChannelImpl start = hardwareMap.get(DigitalChannelImpl.class, "start");

        MaxBoticsArray array = new MaxBoticsArray(start, front, back, left, right);
        array.init();


        waitForStart();
        int pin = 0;
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("distances fblr", Arrays.toString(array.getDistances(DistanceUnit.INCH)));
            if(array.getStartPin().getState()) pin++;
            telemetry.addData("startpin?", array.getStartPin().getState());
            telemetry.addData("startpin pulses", pin);
            telemetry.update();
        }
        requestOpModeStop();
    }
}
