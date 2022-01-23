package org.firstinspires.ftc.teamcode.Development;

import android.icu.text.Transliterator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.Controllers.CupFinder;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

@Autonomous
public class MainAutoV1 extends TeleOPV1 {

    public static Pose2d startPose = new Pose2d(-41, -63, Math.toRadians(90));

    public static Trajectory toDuck, toDuck2, toHub, toDepot, toDepot2;

    public enum State {
        init, toDuck, spinDuck, toHub, deposit, toDepot
    }

    public State state = State.init;

    long duckSpinTimer = 0;

    long duckSpinTime = 5000;

    boolean go = false;

    CupFinder.PositionEnum position = CupFinder.PositionEnum.UNKNOWN;


    @Override
    public void init() {
        super.init();

        webcam:
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


            pipeline = new CupFinder(webcam);
            pipeline.pipelineStageToDisplay = CupFinder.PipelineStages.OUTPUTWITHBOUNDINGRECT;
            webcam.setPipeline(pipeline);

            FtcDashboard.getInstance().startCameraStream(webcam, 10);



            // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
            // out when the RC activity is in portrait. We do our actual image processing assuming
            // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("CAMERA ERROR", errorCode);
                }
            });


        }

        armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armController.setPower(ArmController.armSetPosPower);

        toDuck = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-65, -48), Math.toRadians(180))
                .addDisplacementMarker(() -> duckWheelMotor2.setPower(duckWheelSpeed * (PoseStorage.alliance == PoseStorage.Alliance.BLUE ? 1 : -1)))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDuck2))
                .build();

        toDuck2 = drive.trajectoryBuilder(toDuck.end())
                .splineToSplineHeading(new Pose2d(-65, -54.5, Math.toRadians(90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.5, DriveConstants.MAX_ANG_VEL*.5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
//                .splineToSplineHeading(new Pose2d(-65, -64.75, Math.toRadians(90)), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*.165, DriveConstants.MAX_ANG_VEL*.165, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                .build();

        toHub = drive.trajectoryBuilder(toDuck2.end())
                .lineToConstantHeading(new Vector2d(-65, -30))
                .splineToSplineHeading(new Pose2d(-34, -24, Math.toRadians(180)), Math.toRadians(0))
                .build();

        toDepot = drive.trajectoryBuilder(toHub.end())
                .splineToLinearHeading(new Pose2d(-62, -36, Math.toRadians(-90)), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDepot2))
                .build();

        toDepot2 = drive.trajectoryBuilder(toDepot.end())
                .forward(21)
                .build();

        telemetry.addData(">", "Actually Initialized!!!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        drive.update();
        double[] distances = drive.getDistLocalizer().getDistances();
        double frontDist = distances[0], backDist = distances[1], leftDist = distances[2], rightDist = distances[3];


        if(leftDist > 50) {
            PoseStorage.startingPosition = PoseStorage.StartingPosition.WAREHOUSE;
            PoseStorage.alliance = PoseStorage.Alliance.BLUE;
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
        } else if (rightDist > 50) {
            PoseStorage.startingPosition = PoseStorage.StartingPosition.WAREHOUSE;
            PoseStorage.alliance = PoseStorage.Alliance.RED;
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        } else if (leftDist > rightDist) {
            PoseStorage.startingPosition = PoseStorage.StartingPosition.DUCK;
            PoseStorage.alliance = PoseStorage.Alliance.BLUE;
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            PoseStorage.startingPosition = PoseStorage.StartingPosition.DUCK;
            PoseStorage.alliance = PoseStorage.Alliance.RED;
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }


        position = pipeline.positionDetected;

        telemetry.addData("looping", "...");
        telemetry.addData("Distances", "Front: %f, Back: %f, Left: %f, Right: %f", frontDist, backDist, leftDist, rightDist);
        telemetry.addData("DistancePulse", drive.getDistLocalizer().getArray().getStartPin().getState());
        telemetry.addData("Position", position.toString());
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();
        telemetry.addData("Busy?", drive.isBusy());
        telemetry.addData("State", state.toString());
        telemetry.addData("bestLocalizer", drive.getCurrentLocalizer().getBestCurrentLocalizer().toString());
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
                    drive.followTrajectoryAsync(toDepot);
                    state = State.toDepot;
                }
            }
            break;

            case toDepot: {
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
