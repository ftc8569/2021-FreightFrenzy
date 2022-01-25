package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.MainAutoV1;
import org.firstinspires.ftc.teamcode.Development.TeleOPV1;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedWarehousePaths {

    public Pose2d startingPose = new Pose2d(7, -63, Math.toRadians(90));

    public TrajectorySequence toHubLeft, toHubCenter,  toHubRight,  intoWarehouseLeft, intoWarehouseCenter, intoWarehouseRight, toHub2, intoWarehouse2, toHub3, intoWarehouse3 ;

    public enum Paths {
        start, toHub, delivering, intoWarehouse, intaking, toHub2, delivering2, intoWarehouse2, toHub3, delivering3, intoWarehouse3;
    }

    public Paths path = Paths.start;

    private int cycle = 0;



    public RedWarehousePaths(SampleMecanumDrive drive) {

        toHubLeft = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armBottomPos))
                .splineToSplineHeading(new Pose2d(-12, -42, Math.toRadians(-90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();



        toHubCenter = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-12, -45, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armMiddlePos))
                .splineToSplineHeading(new Pose2d(-12, -39, Math.toRadians(-90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-13.5, -47, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToSplineHeading(new Pose2d(-13.5, -42.75, Math.toRadians(-90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5,
                                DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.3))
                .build();

        intoWarehouseLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToLinearHeading(new Pose2d(12, -68, Math.toRadians(3)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToConstantHeading(new Vector2d(36, -68), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(60, -68), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))                .build();

        intoWarehouseCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToLinearHeading(new Pose2d(12, -68, Math.toRadians(3)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToConstantHeading(new Vector2d(36, -68), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(50, -68), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))                .build();

        intoWarehouseRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToLinearHeading(new Pose2d(0, -68, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(27, -67, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(40, -66), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();




        toHub2 = drive.trajectorySequenceBuilder(intoWarehouseRight.end().minus(new Pose2d(0, 0, 0)))
                .lineToConstantHeading(new Vector2d(12, -67))
                .splineToLinearHeading(new Pose2d(-14, -53, Math.toRadians(-90)), Math.toRadians(135))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-14, -47, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> MainAutoV1.intakeOn = false)
                .splineToConstantHeading(new Vector2d(-14, -42.75), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5,
                                DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.4))
                .build();

        intoWarehouse2 = drive.trajectorySequenceBuilder(toHub2.end())
                .splineToLinearHeading(new Pose2d(0, -68, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(27, -67, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(56, -66), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        toHub3 = drive.trajectorySequenceBuilder(intoWarehouse2.end().minus(new Pose2d(0, 0, 0)))
                .lineToConstantHeading(new Vector2d(12, -67))
                .splineToLinearHeading(new Pose2d(-12.5, -53, Math.toRadians(-90)), Math.toRadians(135))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-12.5, -45, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> MainAutoV1.intakeOn = false)
                .splineToConstantHeading(new Vector2d(-12.5, -41.5), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouse3 = drive.trajectorySequenceBuilder(toHub3.end())
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToLinearHeading(new Pose2d(0, -50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker( () -> drive.setOdometry(true))
                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(0))
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
