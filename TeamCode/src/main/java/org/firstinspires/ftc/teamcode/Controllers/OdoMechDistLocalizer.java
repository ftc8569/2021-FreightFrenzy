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
    protected DistanceSensorArrayLocalizer distLocalizer;
    protected OdoRetractionController odoRetractionController;
    private Localizer bestCurrentLocalizer = odoLocalizer;

    public Pose2d   bestPoseEstimate = new Pose2d(),
                    odoEstimate = new Pose2d(),
                    wheelEstimate = new Pose2d(),
                    distEstimate = new Pose2d();

    public OdoMechDistLocalizer(SampleMecanumDrive drive, HardwareMap hardwareMap,
                                TwoWheelTrackingLocalizer odoLocalizer,
                                MecanumDrive.MecanumLocalizer wheelLocalizer,
                                DistanceSensorArrayLocalizer distLocalizer, OdoRetractionController odoRetractionController) {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.odoLocalizer = odoLocalizer;
        this.wheelLocalizer = wheelLocalizer;
        this.distLocalizer = distLocalizer;
        this.odoRetractionController = odoRetractionController;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return bestPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        distLocalizer.setPoseEstimate(bestPoseEstimate);
        odoLocalizer.setPoseEstimate(bestPoseEstimate);
        wheelLocalizer.setPoseEstimate(bestPoseEstimate);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return odoRetractionController.getUp() ? wheelLocalizer.getPoseVelocity() : odoLocalizer.getPoseVelocity();
    }

    @Override
    public void update() {
        odoLocalizer.update();
        odoEstimate = odoLocalizer.getPoseEstimate();

        wheelLocalizer.update();
        wheelEstimate = wheelLocalizer.getPoseEstimate();

        distLocalizer.setPoseEstimate(bestPoseEstimate);
        distLocalizer.update();
        distEstimate = distLocalizer.getPoseEstimate();

        if(!odoRetractionController.getUp()) {
            if(!distEstimate.equals(new Pose2d())) {
                bestCurrentLocalizer = distLocalizer;
            } else bestCurrentLocalizer = odoLocalizer;
        } else bestCurrentLocalizer = wheelLocalizer;

        bestPoseEstimate = bestCurrentLocalizer.getPoseEstimate();

        distLocalizer.setPoseEstimate(bestPoseEstimate);
        odoLocalizer.setPoseEstimate(bestPoseEstimate);
        wheelLocalizer.setPoseEstimate(bestPoseEstimate);
    }

    public void setOdometry(boolean up) {
        odoRetractionController.set(up);
    }
}
