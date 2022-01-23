package org.firstinspires.ftc.teamcode.Development;

import android.icu.text.Transliterator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.Controllers.CupFinder;
import org.firstinspires.ftc.teamcode.Development.AutoPaths.RedWarehousePaths;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

@Autonomous
public class MainAutoV1 extends TeleOPV1 {

    public static Pose2d startPose = new Pose2d(-41, -63, Math.toRadians(90));


    CupFinder.PositionEnum position = CupFinder.PositionEnum.UNKNOWN;

    RedWarehousePaths redWarehousePaths;

    ElapsedTime deliveryTimer = new ElapsedTime();

    public boolean webcamStreaming = false;


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
        webcamStreaming = true;

        armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armController.setPower(ArmController.armSetPosPower);

        paths: {
            redWarehousePaths = new RedWarehousePaths(drive);
        }


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
        if(webcamStreaming) {
            webcam.closeCameraDeviceAsync(()->{});
            FtcDashboard.getInstance().stopCameraStream();
            webcamStreaming = false;
        }
        drive.update();
        telemetry.addData("Busy?", drive.isBusy());
        telemetry.addData("bestLocalizer", drive.getCurrentLocalizer().getBestCurrentLocalizer().toString());
        telemetry.addData("pose", drive.getPoseEstimate().toString());

        PoseStorage.endPose = drive.getPoseEstimate();
        PoseStorage.armPos = armController.getPosition();

        switch (PoseStorage.startingPosition) {
            case WAREHOUSE: {
                switch (PoseStorage.alliance) {
                    case RED: {
                        switch (redWarehousePaths.path) {
                            case start: {
                                drive.setPoseEstimate(redWarehousePaths.startingPose);
                                switch (position) {
                                    case LEFT:
                                        drive.followTrajectorySequenceAsync(redWarehousePaths.toHubLeft);
                                        break;

                                    case CENTER:
                                        drive.followTrajectorySequenceAsync(redWarehousePaths.toHubCenter);
                                        break;

                                    case RIGHT:
                                        drive.followTrajectorySequenceAsync(redWarehousePaths.toHubRight);
                                        break;
                                }
                                redWarehousePaths.setPath(RedWarehousePaths.Paths.toHub);
                                break;
                            }
                            case toHub: {
                                if(!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.delivering);
                                    deliveryTimer.reset();
                                    armServo.setPosition(armServoOpenPos);
                                }
                                break;
                            }
                            case delivering: {
                                if(deliveryTimer.seconds() > 2) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.intoWarehouse);
                                    armServo.setPosition(armServoShutPos);
                                    switch (position) {
                                        case LEFT:
                                            drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouseLeft);
                                            break;

                                        case CENTER:
                                            drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouseCenter);
                                            break;

                                        case RIGHT:
                                            drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouseRight);
                                            break;
                                    }
                                }
                                break;
                            }
                            case intoWarehouse: {
                                if(!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.toHub2);
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.toHub2);
                                    intakeMotor.setPower(intakeOutSpeed);
                                }
                                break;
                            }
                            case toHub2: {
                                if(!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.delivering2);
                                    armServo.setPosition(armServoOpenPos);
                                    deliveryTimer.reset();
                                }
                                break;
                            }
                            case delivering2: {
                                if(deliveryTimer.seconds() > 2) {
                                    armServo.setPosition(armServoShutPos);
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.intoWarehouse2);
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouse2);
                                }
                                break;

                            }
                            case intoWarehouse2: {
                                if(!drive.isBusy()) {
                                    requestOpModeStop();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
