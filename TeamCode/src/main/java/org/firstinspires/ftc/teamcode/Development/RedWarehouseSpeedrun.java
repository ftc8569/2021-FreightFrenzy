package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class RedWarehouseSpeedrun extends TeleOPV1 {

    public HashMap<String, Trajectory> segments = new HashMap<>();

    public static Pose2d startPose = new Pose2d(-38.5, -83, Math.toRadians(180));

    public static Trajectory toDuck, toDuck2, toHub, toDepot, toDepot2;

    public enum State {
        init, toDuck, spinDuck, toHub, deposit, toDepot
    }

    public State state = State.init;

    long duckSpinTimer = 0;

    long duckSpinTime = 5000;

    boolean go = false;


    @Override
    public void init() {
        super.init();

        PoseStorage.alliance = PoseStorage.Alliance.RED;

        drive.setPoseEstimate(startPose);

        armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armController.setPower(ArmController.armSetPosPower);

        toDuck = drive.trajectoryBuilder(startPose)
                .back(35)
                .build();

        toDuck2 = drive.trajectoryBuilder(toDuck.end())
                .back(8.5, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(2, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        toHub = drive.trajectoryBuilder(toDuck2.end())
                .lineToConstantHeading(new Vector2d(-66, -46))
                .splineToSplineHeading(new Pose2d(-36, -46, Math.toRadians(180)), Math.toRadians(0))
                .build();

        toDepot = drive.trajectoryBuilder(toHub.end())
                .splineToLinearHeading(new Pose2d(-72, -37, Math.toRadians(-90)), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDepot2))
                .build();

        toDepot2 = drive.trajectoryBuilder(toDepot.end())
                .forward(25)
                .build();

        telemetry.addData(">", "Actually Initialized!!!");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();
        telemetry.addData("Busy?", drive.isBusy());
        telemetry.addData("State", state.toString());

        PoseStorage.endPose = drive.getPoseEstimate();
        switch(state){
            case init: {
                drive.followTrajectoryAsync(toDuck);
                state = State.toDuck;
            }
            break;

            case toDuck: {
                if(!drive.isBusy()) {
                    requestOpModeStop();
                }
            }
            break;
//
    }}}

