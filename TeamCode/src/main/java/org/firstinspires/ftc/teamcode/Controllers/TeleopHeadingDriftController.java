package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.checkerframework.checker.units.qual.K;

/**
 * A simple P based controller to try to combat axial drift due to wheel/motor/weight inconsistencies.
 * Currently still a prototype and probably has the opportunity to break. In the case of unwanted
 * movement, tapping your rotation button should reset the heading error and things should work
 * again. The idea of the controller is that while you are not pressing any buttons for rotation,
 * your robot should not be rotating, so we can correct any drift with a feedback controller.
 */
public class TeleopHeadingDriftController {
    Pose2d currentPose;
    Pose2d targetPose;
    double poseTolerance;
    boolean lastTurning = false;
    double Kp;
    boolean enabled = true;


    public TeleopHeadingDriftController(double poseTolerance, double Kp) {
        this.poseTolerance = poseTolerance;
        this.Kp = Kp;
    }

    public Pose2d control(Pose2d currentPose, Pose2d drivepower) {
        this.currentPose = currentPose;
        double turn = drivepower.getHeading();
        boolean turning = Math.abs(turn) > .05;
        if(turning) {
            return drivepower;
        } else {
            if(lastTurning) {
                targetPose = currentPose;
                lastTurning = false;
                return drivepower;
            } else {
                double x = currentPose.getHeading(), //someone else used x in y in their code and
                        // this made it easier for me to figure out which goes where so it's staying
                        y = targetPose.getHeading();
                double error = Math.atan2(Math.sin(x-y), Math.cos(x-y));
                if(Math.abs(Math.toDegrees(error)) >= poseTolerance) return drivepower.plus(new Pose2d(0,0,Math.toDegrees(error) * Kp));
                // possibly * -1 because now that I think about it I think this is actually x and y
                    // swapped so instead of doing rotation-controller.control() in setting your
                    // power it will be +
                else return drivepower;
            }
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean getEnabled() {
        return enabled;
    }
}
