package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A class to be used to cleanly and easily control intakes and keep thier code out of your programs.
 * While most of these methods could reasonably be used to control most mechanisms, the choices are
 * optimised to what I have found us to use for intakes.
 */
public class IntakeController {
    private IntakeHardware hardware; //The initalizer we will use to initialize and control the intake
    private HardwareMap hw; // not strictly necessary right now but as we add build out more methods
                            // it might become necessary
    private double power = 0; // the current power the intake is set to
    private boolean enabled = false;

    /**
     * Creates a IntakeController intstance. It is necessary to implement your own version of
     * IntakeInitializer for your own hardware, and obviously to pass the hardwaremap. Make sure to
     * pass yourInitializer.class as in order to continue controlling your hardware we need to save
     * an instance of the class. Maybe a slightly dubious way to implement this but it works.
     * @param hardware your initalizer class that you implement so that it works corectly for
     *                    your hardware.
     * @param hw the hardware map from your opmode so that we can give it to the initalizer to be
     *           able to find your devices.
     */
    public IntakeController(Class<? extends IntakeHardware> hardware, HardwareMap hw) {
        this.hw = hw;
        try {
            this.hardware = hardware.newInstance();
            this.hardware.init(hw);
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public void setPower(double power) {
        hardware.set(power);
        this.enabled = power != 0;
        this.power = power;
    }

    public double getPower() {
        return enabled ? power : 0;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        hardware.set(enabled ? power : 0);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void toggle() {
        hardware.set(enabled ? 0 : power);
        enabled = !enabled;
    }

    public void reverse() {
       power *= -1;
       hardware.set(power);
    }

    public void stop() {
        hardware.set(0.0);
        power = 0;
        enabled = false;
    }


}
