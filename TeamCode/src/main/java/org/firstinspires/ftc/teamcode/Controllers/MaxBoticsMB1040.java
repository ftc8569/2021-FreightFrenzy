package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MaxBoticsMB1040 implements DistanceSensor {
    private final AnalogInput input;
    private final double scaleFactorPerInch = 3.3/512;

    public MaxBoticsMB1040(AnalogInput input) {
        this.input = input;
    }
    @Override
    public double getDistance(DistanceUnit unit) {
        return  2 * unit.fromInches(input.getVoltage() / scaleFactorPerInch);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotics MB-1040-000";
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
