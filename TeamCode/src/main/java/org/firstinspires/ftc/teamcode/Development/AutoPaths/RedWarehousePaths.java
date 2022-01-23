package org.firstinspires.ftc.teamcode.Development.AutoPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.TeleOPV1;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedWarehousePaths {

    public Pose2d startingPose = new Pose2d(7, -63, Math.toRadians(90));

    public TrajectorySequence toHubLeft, toHubCenter,  toHubRight,  intoWarehouseLeft, intoWarehouseCenter, intoWarehouseRight, toHub2, intoWarehouse2;

    public enum Paths {
        start, toHub, delivering, intoWarehouse, intaking, toHub2, delivering2, intoWarehouse2
    }

    public Paths path = Paths.start;



    public RedWarehousePaths(SampleMecanumDrive drive) {

        toHubLeft = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armBottomPos))
                .splineToSplineHeading(new Pose2d(-12, -42, Math.toRadians(-90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25, DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();



        toHubCenter = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-12, -45, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armMiddlePos))
                .splineToSplineHeading(new Pose2d(-12, -39, Math.toRadians(-90)), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25, DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        toHubRight = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(-12, -45, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armTopPos))
                .splineToSplineHeading(new Pose2d(-12, -41, Math.toRadians(-90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouseLeft = drive.trajectorySequenceBuilder(toHubLeft.end())
                .splineToLinearHeading(new Pose2d(12, -66, 0), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armStartPos))
                .splineToSplineHeading(new Pose2d(36, -66, 0), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.intakeMotor.setPower(TeleOPV1.intakeSpeed))
                .splineToSplineHeading(new Pose2d(48, -66, 0), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                        DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouseCenter = drive.trajectorySequenceBuilder(toHubCenter.end())
                .splineToLinearHeading(new Pose2d(12, -66, 0), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armStartPos))
                .splineToSplineHeading(new Pose2d(36, -66, 0), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.intakeMotor.setPower(TeleOPV1.intakeSpeed))
                .splineToSplineHeading(new Pose2d(48, -66, 0), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouseRight = drive.trajectorySequenceBuilder(toHubRight.end())
                .splineToLinearHeading(new Pose2d(12, -66, 0), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armStartPos))
                .splineToConstantHeading(new Vector2d(36, -66), Math.toRadians(0))
                .addDisplacementMarker(() -> TeleOPV1.intakeMotor.setPower(TeleOPV1.intakeSpeed))
                .splineToConstantHeading(new Vector2d(48, -66), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        toHub2 = drive.trajectorySequenceBuilder(intoWarehouseLeft.end())
                .lineToConstantHeading(new Vector2d(-12, -66))
                .addDisplacementMarker(() -> TeleOPV1.armController.setPosition((int) TeleOPV1.armTopPos))
                .splineToLinearHeading(new Pose2d(-12, -45, Math.toRadians(-90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12, -41, Math.toRadians(-90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.25,
                                DriveConstants.MAX_ANG_VEL*.25, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.25))
                .build();

        intoWarehouse2 = drive.trajectorySequenceBuilder(toHub2.end())
                .splineToLinearHeading(new Pose2d(-12, -65, 0), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(36, -65), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -36), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(63, -42, Math.toRadians(-90)), Math.toRadians(-90))
                .build();





    }

    public void setPath(Paths path) {
        this.path = path;
    }

}
