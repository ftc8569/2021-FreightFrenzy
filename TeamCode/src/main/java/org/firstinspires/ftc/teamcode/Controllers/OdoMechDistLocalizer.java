package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * A combined localizer that uses the mecanum and the two wheel odometry localizers but also uses a
 * maxbotics mb1040 ultrasonic distance sensor on each side of the robot.
 */
public class OdoMechDistLocalizer implements Localizer {
    protected SampleMecanumDrive drive;
    protected  HardwareMap hardwareMap;
    protected  TwoWheelTrackingLocalizer odoLocalizer;
    protected MecanumDrive.MecanumLocalizer wheelLocalizer;
    protected MaxBoticsArray distArray;

    public Pose2d   bestPoseEstimate = new Pose2d(),
                    odoEstimate = new Pose2d(),
                    wheelEstimate = new Pose2d(),
                    distEstimate = new Pose2d();

    public OdoMechDistLocalizer(SampleMecanumDrive drive, HardwareMap hardwareMap,
                                TwoWheelTrackingLocalizer odoLocalizer,
                                MecanumDrive.MecanumLocalizer wheelLocalizer,
                                MaxBoticsArray distArray) {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.odoLocalizer = odoLocalizer;
        this.wheelLocalizer = wheelLocalizer;
        this.distArray = distArray;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        odoLocalizer.update();
        odoEstimate = odoLocalizer.getPoseEstimate();

        wheelLocalizer.update();
        wheelEstimate = wheelLocalizer.getPoseEstimate();


        odoLocalizer.setPoseEstimate(bestPoseEstimate);
        wheelLocalizer.setPoseEstimate(bestPoseEstimate);
    }
}
