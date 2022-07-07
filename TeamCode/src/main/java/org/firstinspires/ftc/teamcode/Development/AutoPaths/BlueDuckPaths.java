package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Development.MainAutoV1;
import org.firstinspires.ftc.teamcode.Development.TeleOPV1;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueDuckPaths {

    public Pose2d startingPose = new Pose2d(-43, 67.2, Math.toRadians(-90));

    public TrajectorySequence toHubLeft, toHubCenter,  toHubRight, toWaitLeft, toWaitCenter, toWaitRight, toWarehouse;

    public enum Paths {
        start, toHub, delivering, toWait, waiting, toWarehouse;
    }

    public Paths path = Paths.start;

    private int cycle = 0;

    private final SampleMecanumDrive drive;


    //All positions just mirrored from red duck auto. Y values and angles are * -1

    public BlueDuckPaths(SampleMecanumDrive drive) {

        this.drive = drive;


        toHubLeft = drive.trajectorySequenceBuilder(startingPose)
                .addDisplacementMarker(.1, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armBottomPos))
                .splineToLinearHeading(new Pose2d(-24, 58, Math.toRadians(110)), Math.toRadians(250),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();


        toHubCenter = drive.trajectorySequenceBuilder(startingPose)
                .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armMiddlePos))
                .splineToLinearHeading(new Pose2d(-20, 54, Math.toRadians(110)), Math.toRadians(250),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(startingPose)
                .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-16, 50, Math.toRadians(110)), Math.toRadians(250),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();


        toWaitLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToLinearHeading(new Pose2d(-36, 75, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toWaitCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToLinearHeading(new Pose2d(-36, 75, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toWaitRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToLinearHeading(new Pose2d(-36, 75, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toWarehouse = drive.trajectorySequenceBuilder(toWaitRight.end())
                .splineToConstantHeading(new Vector2d(30, 75), 0)
                .build();

    }

    public void setPath(Paths path) {
        this.path = path;
    }

    public Paths getPath() {
        return path;
    }

    public int getCycle() {
        return cycle;
    }

    public void addCycle() {
        this.cycle++;
    }
}
