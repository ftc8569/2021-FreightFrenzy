package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Development.MainAutoV1;
import org.firstinspires.ftc.teamcode.Development.TeleOPV1;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedNewDuckPaths {

    public Pose2d startingPose = new Pose2d(-91, -67.2, Math.toRadians(90));

    public TrajectorySequence toHubLeft, toHubCenter,  toHubRight, toDuckLeft, toDuckCenter, toDuckRight, toWarehouse;

    public enum Paths {
        start, toHub, delivering, toDuck, spinDuck, toWarehouse;
    }

    public Paths path = Paths.start;

    private int cycle = 0;

    private final SampleMecanumDrive drive;


    //All positions just mirrored from red duck auto. Y values and angles are * -1

    public RedNewDuckPaths(SampleMecanumDrive drive) {

        this.drive = drive;


        toHubLeft = drive.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-114, -43), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-95, -43))
                .lineToConstantHeading(new Vector2d(-65, -43),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .4,
                                DriveConstants.MAX_ANG_VEL * .4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .splineToLinearHeading(new Pose2d(-76, -29, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -4, Math.toRadians(45)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToConstantHeading(new Vector2d(-44, -4), 0)
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armBottomPos))
                .splineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(135)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-36, -10, Math.toRadians(135)), Math.toRadians(-45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();


        toHubCenter = drive.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-114, -43), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-100, -43))
                .lineToConstantHeading(new Vector2d(-65, -43),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .4,
                                DriveConstants.MAX_ANG_VEL * .4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .splineToLinearHeading(new Pose2d(-76, -29, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -4, Math.toRadians(45)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToConstantHeading(new Vector2d(-44, -4), 0)
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armMiddlePos))
                .splineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(135)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(135)), Math.toRadians(-45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-114, -43), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-65, -43),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .4,
                                DriveConstants.MAX_ANG_VEL * .4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .splineToLinearHeading(new Pose2d(-76, -29, Math.toRadians(45)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -4, Math.toRadians(45)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToConstantHeading(new Vector2d(-44, -4), 0)
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-34, 0, Math.toRadians(135)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-28, -20, Math.toRadians(135)), Math.toRadians(-45),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();



    }

    public void init() {
        toDuckLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(135)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(45)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-75, -34, Math.toRadians(45)), Math.toRadians(-135),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToLinearHeading(new Pose2d(-110, -66, Math.toRadians(0)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-110, -68),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();

        toDuckCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(135)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(45)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-75, -34, Math.toRadians(45)), Math.toRadians(-135),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToLinearHeading(new Pose2d(-110, -66, Math.toRadians(0)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-110, -68),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .build();

        toDuckRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(135)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(45)), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-78, -34), Math.toRadians(-134.9),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .5))
                .splineToLinearHeading(new Pose2d(-110, -65, Math.toRadians(0)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-110, -67),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .5,
                                DriveConstants.MAX_ANG_VEL * .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .25))
                .build();

        toWarehouse = drive.trajectorySequenceBuilder(toDuckRight.end())
                .splineToLinearHeading(new Pose2d(-112, -30, Math.toRadians(90)), Math.toRadians(90))
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
