package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.DigitalIoDeviceConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class MaxBoticsArray {
    private MaxBoticsMB1040 frontSensor, backSensor, leftSensor, rightSensor;
    private DigitalChannelImpl startPin;

    public static boolean isLooping = false;

    public MaxBoticsArray(DigitalChannel startPin, AnalogInput frontSensor, AnalogInput backSensor, AnalogInput leftSensor, AnalogInput rightSensor) {
        this.frontSensor = new MaxBoticsMB1040(frontSensor);
        this.backSensor = new MaxBoticsMB1040(backSensor);
        this.leftSensor = new MaxBoticsMB1040(leftSensor);
        this.rightSensor = new MaxBoticsMB1040(rightSensor);
        this.startPin = (DigitalChannelImpl) startPin;

        if(!isLooping) {
            ElapsedTime timer = new ElapsedTime();

            startPin.setMode(DigitalChannel.Mode.OUTPUT);
            startPin.setState(true);
            timer.reset();

            while (timer.milliseconds() < 1) {

            }
            startPin.setState(false);
            startPin.setMode(DigitalChannel.Mode.INPUT);
        }
        isLooping = true;
    }

    public double[] getDistances(DistanceUnit unit) {
        return new double[]{frontSensor.getDistance(unit), backSensor.getDistance(unit), leftSensor.getDistance(unit), rightSensor.getDistance(unit)};
    }

    public MaxBoticsMB1040[] getSensors() {
        return new MaxBoticsMB1040[]{frontSensor, backSensor, leftSensor, rightSensor};
    }

    public DigitalChannelImpl getStartPin() {
        return startPin;
    }
}
