package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;

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
    private Localizer bestCurrentLocalizer = odoLocalizer, lastBestLocalizer = odoLocalizer;

    public Pose2d   bestPoseEstimate = new Pose2d(),
                    odoEstimate = new Pose2d(),
                    wheelEstimate = new Pose2d(),
                    distEstimate = new Pose2d();

    public double heading = 0;

    private double offset = 0;


    public OdoMechDistLocalizer(@NotNull SampleMecanumDrive drive, @NotNull TwoWheelTrackingLocalizer odoLocalizer,
                                @NotNull MecanumDrive.MecanumLocalizer wheelLocalizer,
                                @NotNull DistanceSensorArrayLocalizer distLocalizer, @NotNull OdoRetractionController odoRetractionController) {
        this.drive = drive;
        this.odoLocalizer = odoLocalizer;
        this.wheelLocalizer = wheelLocalizer;
        this.distLocalizer = distLocalizer;
        this.odoRetractionController = odoRetractionController;
        bestCurrentLocalizer = odoLocalizer;
        lastBestLocalizer = odoLocalizer;

        heading = drive.getRawExternalHeading();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return bestPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        bestPoseEstimate = pose2d;
        offset = Angle.normDelta(heading - pose2d.getHeading());
        heading = Angle.normDelta(drive.getRawExternalHeading() - offset);
        distLocalizer.setPoseEstimate(new Pose2d(pose2d.getX(), pose2d.getY(), heading));
        odoLocalizer.setPoseEstimate(pose2d);
        wheelLocalizer.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2d poseVelocity = new Pose2d();
        if(!odoRetractionController.isUp() && odoLocalizer.getPoseVelocity() != null) {
            poseVelocity = odoLocalizer.getPoseVelocity();
        } else if (wheelLocalizer.getPoseVelocity() != null) {
            poseVelocity = wheelLocalizer.getPoseVelocity();
        }
        return poseVelocity;
    }

    @Override
    public void update() {
        heading = Angle.normDelta(drive.getRawExternalHeading() - offset);

        odoLocalizer.update();
        odoEstimate = odoLocalizer.getPoseEstimate();

        wheelLocalizer.update();
        wheelEstimate = wheelLocalizer.getPoseEstimate();

        Pose2d poseVelocity = new Pose2d();
        if(!odoRetractionController.isUp() && odoLocalizer.getPoseVelocity() != null) {
            poseVelocity = odoLocalizer.getPoseVelocity();
        } else if (wheelLocalizer.getPoseVelocity() != null) {
            poseVelocity = wheelLocalizer.getPoseVelocity();
        }
        distLocalizer.setPoseVelocity(poseVelocity);
        distLocalizer.setPoseEstimate(new Pose2d(bestPoseEstimate.getX(), bestPoseEstimate.getY(), heading));
        distLocalizer.update();
        distEstimate = distLocalizer.getPoseEstimate();

        if(!odoRetractionController.isUp()) {
//            if(!distEstimate.equals(new Pose2d())) {
//                bestCurrentLocalizer = distLocalizer;
//            } else
            bestCurrentLocalizer = odoLocalizer;
        } else bestCurrentLocalizer = wheelLocalizer;

        if(bestCurrentLocalizer == distLocalizer) {
            distLocalizer.updateHeading(heading);
        } else if(!lastBestLocalizer.equals(bestCurrentLocalizer)) {
            odoLocalizer.setPoseEstimate(bestPoseEstimate);
            wheelLocalizer.setPoseEstimate(bestPoseEstimate);
        }



        bestPoseEstimate = bestCurrentLocalizer.getPoseEstimate();

//        distLocalizer.setPoseEstimate(bestPoseEstimate);
        lastBestLocalizer = bestCurrentLocalizer;
    }

    public void setOdometry(boolean up) {
        odoRetractionController.set(up);
    }

    public Localizer getBestCurrentLocalizer() {
        return bestCurrentLocalizer;
    }

    public double[] getDistances() {
        return distLocalizer.getDistances();
    }

    public Pose2d getDistEstimate() {
        return distEstimate;
    }

    public Pose2d getOdoEstimate() {
        return odoEstimate;
    }

    public Pose2d getWheelEstimate() {
        return wheelEstimate;
    }
}
