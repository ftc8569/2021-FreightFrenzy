package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

@Autonomous
public class RedDuckAutoWarehouse extends TeleOPV1 {

    public HashMap<String, Trajectory> segments = new HashMap<>();

    public static Pose2d startPose = new Pose2d(-41, -63, Math.toRadians(90));

    public static Trajectory toDuck, toDuck2, toHub, toWarehouse, toDepot2;

    public enum State {
        init, toDuck, spinDuck, toHub, deposit, toWarehouse
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
                .splineToConstantHeading(new Vector2d(-65, -48), Math.toRadians(180))
                .addDisplacementMarker(() -> duckWheelMotor2.setPower(duckWheelSpeed * (PoseStorage.alliance == PoseStorage.Alliance.BLUE ? 1 : -1)))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDuck2))
                .build();

        toDuck2 = drive.trajectoryBuilder(toDuck.end())
                .splineToSplineHeading(new Pose2d(-65, -52.75, Math.toRadians(90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
//                .splineToSplineHeading(new Pose2d(-65, -64.75, Math.toRadians(90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.165, DriveConstants.MAX_ANG_VEL*.165, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();

        toHub = drive.trajectoryBuilder(toDuck2.end())
                .lineToConstantHeading(new Vector2d(-65, -30))
                .splineToSplineHeading(new Pose2d(-34, -24, Math.toRadians(180)), Math.toRadians(0))
                .build();

        toWarehouse = drive.trajectoryBuilder(toHub.end())
                .splineToSplineHeading(new Pose2d(-60, -48, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-36,-62, 0), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36, -65, 0), Math.toRadians(5))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDepot2))
                .build();

        toDepot2 = drive.trajectoryBuilder(toWarehouse.end())
                .forward(21)
                .build();

        telemetry.addData(">", "Actually Initialized!!!");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();
        telemetry.addData("Busy?", drive.isBusy());
        telemetry.addData("State", state.toString());
//        telemetry.addData("bestLocalizer", drive.getCurrentLocalizer().getBestCurrentLocalizer().toString());
        telemetry.addData("pose", drive.getPoseEstimate().toString());

        PoseStorage.endPose = drive.getPoseEstimate();
        PoseStorage.armPos = armController.getPosition();

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
                    duckWheelMotor2.setPower(0);
//                    requestOpModeStop();
                    drive.followTrajectoryAsync(toHub);
//                    intakeMotor.setPower(intakeSpeed);
                    state = State.toHub;
                }
            }
            break;

            case toHub: {
                if(!drive.isBusy()) {
                    armController.setPosition((int) armTopPos);
//                    armServo.setPosition(armServoOpenPos);
                    state = State.deposit;
                }
            }
            break;

            case deposit: {
                if(Math.abs(armController.getPosition() - armTopPos*.6) < ArmHardware2021.targetPosTolerance) {
                    armServo.setPosition(armServoOpenPos);
                }
                if(Math.abs(armController.getPosition() - armTopPos) < ArmHardware2021.targetPosTolerance) {
//                    armServo.setPosition(armServoShutPos);
                    armController.setPosition((int) armStartPos);
//                    intakeMotor.setPower(0);
                    drive.followTrajectoryAsync(toWarehouse);
                    state = State.toWarehouse;
                }
            }
            break;

            case toWarehouse: {
                if(Math.abs(armController.getPosition() - armTopPos*.6) < ArmHardware2021.targetPosTolerance) {
                    armServo.setPosition(armServoShutPos);
                }
                if(!drive.isBusy()) {
                    armController.setPower(0);
                    requestOpModeStop();
                }
            }
            break;
        }
    }
}
