package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Controllers.CupFinder;
import org.firstinspires.ftc.teamcode.Development.MainAutoV1;
import org.firstinspires.ftc.teamcode.Development.PoseStorage;
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
                .addDisplacementMarker(0, 0.25, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armBottomPos))
                .splineToSplineHeading(new Pose2d(3, -46, Math.toRadians(-45)), Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();



        toHubCenter = drive.trajectorySequenceBuilder(startingPose)
                .addDisplacementMarker(.1, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armMiddlePos))
                .splineToSplineHeading(new Pose2d(.75, -45.75, Math.toRadians(-45)), Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(startingPose)
                .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToSplineHeading(new Pose2d(-5.25, -42.75, Math.toRadians(-45)), Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();

        intoWarehouseLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(22, -65.5, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(46, -65.5), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouseCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToLinearHeading(new Pose2d(0, -65.25, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(22, -65.25, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(46, -65.5), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouseRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(22, -65.5, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(46, -65.5), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();




        toHub2 = drive.trajectorySequenceBuilder(intoWarehouseRight.end().minus(new Pose2d(0, 0, 0)))
                .lineToConstantHeading(new Vector2d(-4, -65))
                .addDisplacementMarker(() -> {
                    if(MainAutoV1.position == CupFinder.PositionEnum.LEFT && PoseStorage.startingPosition == PoseStorage.StartingPosition.WAREHOUSE) {
                        drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(-4, -.5, 0)));
                    }
                })
                .addDisplacementMarker(.2, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = true;
                })
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-.5, -42, Math.toRadians(-45)), Math.toRadians(115),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .addDisplacementMarker(.9, 0, () -> MainAutoV1.intakeOn = false)
                .build();

        intoWarehouse2 = drive.trajectorySequenceBuilder(toHub2.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(27, -65.5, 0), Math.toRadians(0))
                .addDisplacementMarker(.3, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = false;
                })
                .splineToConstantHeading(new Vector2d(53, -65.5), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        toHub3 = drive.trajectorySequenceBuilder(intoWarehouse2.end().minus(new Pose2d(0, 0, 0)))
                .lineToLinearHeading(new Pose2d(-2, -65.5, Math.toRadians(-1)))
                .addDisplacementMarker(() -> {
                    if(MainAutoV1.position == CupFinder.PositionEnum.LEFT && PoseStorage.startingPosition == PoseStorage.StartingPosition.WAREHOUSE) {
                        drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(-2, 0, 0)));
                    }
                })
                .addDisplacementMarker(.2, 0, () -> {
                    MainAutoV1.intakeOn = true;
                    MainAutoV1.intakeReverse = true;
                })
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                .splineToLinearHeading(new Pose2d(.5, -42, Math.toRadians(-45)), Math.toRadians(115),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                        DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .addDisplacementMarker(.9, 0, () -> MainAutoV1.intakeOn = false)
                .build();

        intoWarehouse3 = drive.trajectorySequenceBuilder(toHub3.end())
                .splineToLinearHeading(new Pose2d(0, -65.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) MainAutoV1.armStartPos))
                .splineToSplineHeading(new Pose2d(36, -65.5, 0), Math.toRadians(0))
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
