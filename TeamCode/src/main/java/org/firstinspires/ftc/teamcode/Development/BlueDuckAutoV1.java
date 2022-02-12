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
public class BlueDuckAutoV1 extends TeleOPV1 {

    public HashMap<String, Trajectory> segments = new HashMap<>();

    public static Pose2d startPose = new Pose2d(-38.5, 83, Math.toRadians(-90));

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

        PoseStorage.alliance = PoseStorage.Alliance.BLUE;

        drive.setPoseEstimate(startPose);

        armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armController.setPower(ArmController.armSetPosPower);

        toDuck = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-69, 78), Math.toRadians(180))
                .addDisplacementMarker(() -> duckWheelMotor.setPower(duckWheelSpeed * (PoseStorage.alliance == PoseStorage.Alliance.BLUE ? 1 : -1)))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDuck2))
                .build();

        toDuck2 = drive.trajectoryBuilder(toDuck.end())
                .back(6.75, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(2, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.165, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        toHub = drive.trajectoryBuilder(toDuck2.end())
                .lineToConstantHeading(new Vector2d(-66, 52))
                .splineToSplineHeading(new Pose2d(-36, 52, Math.toRadians(180)), Math.toRadians(0))
                .build();

        toDepot = drive.trajectoryBuilder(toHub.end())
                .splineToLinearHeading(new Pose2d(-72, 42, Math.toRadians(90)), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDepot2))
                .build();

        toDepot2 = drive.trajectoryBuilder(toDepot.end())
                .forward(27)
                .build();

        telemetry.addData(">", "Actually Initialized!!!");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();
        telemetry.addData("Busy?", drive.isBusy());
        telemetry.addData("State", state.toString());

        switch(state){
            case init: {
                drive.followTrajectoryAsync(toDuck);
                state = State.toDuck;
            }
            break;

            case toDuck: {
                if(!drive.isBusy()) {
                    state = State.spinDuck;
                    duckSpinTimer = System.currentTimeMillis();
                }
            }
            break;
//
            case spinDuck: {
                if(System.currentTimeMillis() - duckSpinTimer > duckSpinTime) {
                    duckWheelMotor.setPower(0);
                    drive.followTrajectoryAsync(toHub);
                    intakeMotor.setPower(intakeSpeed);
                    state = State.toHub;
                }
            }
            break;

            case toHub: {
                if(!drive.isBusy()) {
                    armController.setPosition((int) armTopPos);
                    state = State.deposit;
                }
            }
            break;

            case deposit: {
                if(Math.abs(armController.getPosition() - armTopPos) < ArmHardware2021.targetPosTolerance) {
                    armController.setPosition((int) armStartPos);
                    intakeMotor.setPower(0);
                    drive.followTrajectoryAsync(toDepot);
                    state = State.toDepot;
                }
            }
            break;

            case toDepot: {
                if(!drive.isBusy()) {
                    armController.setPower(0);
                    PoseStorage.endPose = drive.getPoseEstimate();
                    requestOpModeStop();
                }
            }
            break;
        }
    }
}
