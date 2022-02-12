package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Controllers.CupFinder;
import org.firstinspires.ftc.teamcode.Development.MainAutoV1;
import org.firstinspires.ftc.teamcode.Development.PoseStorage;
import org.firstinspires.ftc.teamcode.Development.TeleOPV1;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueDuckPaths {

    public Pose2d startingPose = new Pose2d(-43, 62.2, Math.toRadians(-90));

    public TrajectorySequence toHubLeft, toDuck, toHubCenter,  toHubRight, toStorageLeft, toStorageCenter, toStorageRight;

    public enum Paths {
        start, toDuck, spinningDuck, toHub, delivering, toStorage;
    }

    public Paths path = Paths.start;

    private int cycle = 0;

    private final SampleMecanumDrive drive;


    //All positions just mirrored from red duck auto. Y values and angles are * -1

    public BlueDuckPaths(SampleMecanumDrive drive) {

        this.drive = drive;

        toDuck = drive.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-65, 48), Math.toRadians(-180))
                .lineToConstantHeading(new Vector2d(-65, 56.5), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        toHubLeft = drive.trajectorySequenceBuilder(toDuck.end())
                .addDisplacementMarker(.1, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armBottomPos))
                .lineToConstantHeading(new Vector2d(-65, 24))
                .splineToLinearHeading(new Pose2d(-42.5, 24, Math.toRadians(-180)), Math.toRadians(-0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();


        toHubCenter = drive.trajectorySequenceBuilder(toDuck.end())
                .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armMiddlePos))
                .lineToConstantHeading(new Vector2d(-65, 24))
                .splineToLinearHeading(new Pose2d(-39, 24, Math.toRadians(-180)), Math.toRadians(-0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(toDuck.end())
                .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .lineToConstantHeading(new Vector2d(-65, 24))
                .splineToLinearHeading(new Pose2d(-30.5, 24, Math.toRadians(-180)), Math.toRadians(-0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();


        toStorageLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToConstantHeading(new Vector2d(-40, 24), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-62, 37, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toStorageCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToConstantHeading(new Vector2d(-40, 24), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-62, 37, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toStorageRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToConstantHeading(new Vector2d(-40, 24), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-62, 37, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
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
