package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;
@Disabled

@Autonomous
public class BlueWarehouseSpeedrun extends TeleOPV1 {

    public HashMap<String, Trajectory> segments = new HashMap<>();

    public static Pose2d startPose = new Pose2d(7, 63, Math.toDegrees(180));

    public static Trajectory toDeposit, toDuck2, toHub, toWarehouse, toWarehouse2;

    public enum State {
        init, toDeposit, deposit, toWarehouse, a, toDepot
    }

    public State state = State.init;

    long duckSpinTimer = 0;

    long duckSpinTime = 5000;

    boolean go = false;


    @Override
    public void init() {
        super.init();

        PoseStorage.alliance = PoseStorage.Alliance.BLUE;

        drive.setPoseEstimate(startPose);

        armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armController.setPower(ArmController.armSetPosPower);

        toDeposit = drive.trajectoryBuilder(startPose)
.back(32)                .build();

        toDuck2 = drive.trajectoryBuilder(toDeposit.end())
                .back(6.75, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(2, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.165, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        toHub = drive.trajectoryBuilder(toDuck2.end())
                .lineToConstantHeading(new Vector2d(-66, 52))
                .splineToSplineHeading(new Pose2d(-36, 52, Math.toRadians(180)), Math.toRadians(0))
                .build();

        toWarehouse = drive.trajectoryBuilder(toHub.end())
                .splineToLinearHeading(new Pose2d(12, 65, Math.toRadians(0)), Math.toRadians(90))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toWarehouse2))
                .build();

        toWarehouse2 = drive.trajectoryBuilder(toWarehouse.end())
                .forward(24)
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
                drive.followTrajectoryAsync(toDeposit);
                state = State.toDeposit;
            }
            break;

            case toDeposit: {
                if(!drive.isBusy()) {
                   requestOpModeStop();
                }
            }
            break;
//
            case deposit: {
                if(Math.abs(armController.getPosition() - armTopPos) < ArmHardware2021.targetPosTolerance) {
                    armController.setPosition((int) armStartPos);
                    intakeMotor.setPower(0);
                    drive.followTrajectoryAsync(toWarehouse);
                    state = State.toDepot;
                }
            }
            break;

            case toWarehouse: {
                if(!drive.isBusy()) {
                    requestOpModeStop();
                }
            }
            break;

        }
    }
}
