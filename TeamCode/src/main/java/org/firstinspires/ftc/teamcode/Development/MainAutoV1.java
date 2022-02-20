package org.firstinspires.ftc.teamcode.Development;

import android.graphics.BlendMode;
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
import org.firstinspires.ftc.teamcode.Controllers.FreightSensorController;
import org.firstinspires.ftc.teamcode.Development.AutoPaths.BlueDuckPaths;
import org.firstinspires.ftc.teamcode.Development.AutoPaths.BlueWarehousePaths;
import org.firstinspires.ftc.teamcode.Development.AutoPaths.RedDuckPaths;
import org.firstinspires.ftc.teamcode.Development.AutoPaths.RedWarehousePaths;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.Development.PoseStorage.Alliance.BLUE;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class MainAutoV1 extends TeleOPV1 {

    public static CupFinder.PositionEnum position = CupFinder.PositionEnum.UNKNOWN;

    RedWarehousePaths redWarehousePaths;

    RedDuckPaths redDuckPaths;

    BlueWarehousePaths blueWarehousePaths;

    BlueDuckPaths blueDuckPaths;

    ElapsedTime deliveryTimer = new ElapsedTime();

    public boolean webcamStreaming = false;

    OpenCvWebcam webcam;

    CupFinder pipeline;

    public final int CYCLE_COUNT = 2;

    public static double armStartPos,
            armTopPos,
            armMiddlePos,
            armBottomPos;

    double[] distances;
    double frontDist, backDist, leftDist, rightDist;

    boolean cameraSet = false;


    @Override
    public void init() {
        PoseStorage.armPos = 0;

        super.init();

        armStartPos = TeleOPV1.armStartPos;
        armTopPos = TeleOPV1.armTopPos;
        armMiddlePos = TeleOPV1.armMiddlePos;
        armBottomPos = TeleOPV1.armBottomPos;

        intakeOn = false;
        intakeReverse = false;

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

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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

        paths:
        {
            redWarehousePaths = new RedWarehousePaths(drive);
            redDuckPaths = new RedDuckPaths(drive);
            blueDuckPaths = new BlueDuckPaths(drive);
            blueWarehousePaths = new BlueWarehousePaths(drive);
        }

        tapePanServo.setPosition(tapePanVisionPos);
        tapeTiltServo.setPosition(tapeTiltVisionPos);

        depositController.hold();

        telemetry.addData(">", "Actually Initialized!!!");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        drive.update();
        Position pos = findPosition();
        PoseStorage.alliance = pos.getAlliance();
        PoseStorage.startingPosition = pos.getPosition();

        switch (PoseStorage.alliance) {
            case RED:
                duckDirection = -1;
                if(PoseStorage.startingPosition == PoseStorage.StartingPosition.WAREHOUSE) {
                    PoseStorage.endPose = redWarehousePaths.startingPose;
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                } else {
                    PoseStorage.endPose = redDuckPaths.startingPose;
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                break;

            case BLUE:
                duckDirection = 1;
                if(PoseStorage.startingPosition == PoseStorage.StartingPosition.WAREHOUSE) {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                    PoseStorage.endPose = blueWarehousePaths.startingPose;
                } else {
                    PoseStorage.endPose = blueDuckPaths.startingPose;
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                break;

            case NEITHER:
                duckDirection = -1;
                break;

        }

        duck.setReverse(duckDirection == -1);

        position = pipeline.positionDetected;

        telemetry.addData("looping", "...");
        telemetry.addData("Distances", "Front: %f, Back: %f, Left: %f, Right: %f", frontDist, backDist, leftDist, rightDist);
        telemetry.addData("DistancePulse", drive.getDistLocalizer().getArray().getStartPin().getState());
        telemetry.addData("Position", position.toString());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (webcamStreaming) {
            webcam.closeCameraDeviceAsync(() -> {
            });
            FtcDashboard.getInstance().stopCameraStream();
            webcamStreaming = false;
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
        if(!cameraSet) {
            tapeTiltServo.setPosition(tapeTiltNormalPos);
            tapePanServo.setPosition(tapePanNormalPos);
            cameraSet = true;
        }
        drive.update();
        duck.update();
        telemetry.addData("Busy?", drive.isBusy());
//        telemetry.addData("bestLocalizer", drive.getCurrentLocalizer().getBestCurrentLocalizer().toString());
        telemetry.addData("pose", drive.getPoseEstimate().toString());
        telemetry.addData("freightRGBA", "%s, %s, %s, %s", FreightSensor.red, FreightSensor.green, FreightSensor.blue, FreightSensor.alpha);
        telemetry.addData("combined Freight", FreightSensor.getSum());
        telemetry.addData("Current Path", redWarehousePaths.getPath().toString());

        PoseStorage.endPose = drive.getPoseEstimate();
        PoseStorage.armPos = armController.getPosition();

        if (intakeOn) intakeMotor.setPower(intakeReverse ? -intakeOutSpeed : intakeSpeed);
        else intakeMotor.setPower(0);


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

                                    case UNKNOWN:
                                    case RIGHT:
                                        drive.followTrajectorySequenceAsync(redWarehousePaths.toHubRight);
                                        break;
                                }
                                redWarehousePaths.setPath(RedWarehousePaths.Paths.toHub);
                                break;
                            }
                            case toHub: {
                                if (!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.delivering);
                                    deliveryTimer.reset();
                                    depositController.set(true);
                                }
                                break;
                            }
                            case delivering: {
                                if (deliveryTimer.seconds() > .75) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.intoWarehouse);
                                    depositController.set(false);
                                    freight = FreightSensorController.Freight.NONE;
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
                                FreightSensor.update();
                                freight= FreightSensor.getFreight();
                                switch (freight) {
                                    case CUBE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                                        break;
                                    case BALL:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                        break;
                                    case NONE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                        break;
                                }

                                if (!drive.isBusy()) {
                                    drive.cancel();
                                    depositController.hold();
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.toHub2);
//                                    redWarehousePaths.generateToHub2();
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.toHub2);
                                }
                                break;
                            }
                            case toHub2: {
                                if (!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.delivering2);
                                    depositController.set(true);
                                    deliveryTimer.reset();

                                }
                                break;
                            }
                            case delivering2: {
                                if (deliveryTimer.seconds() > .75) {
                                    depositController.set(false);
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.intoWarehouse2);
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouse2);
                                }
                                break;
                            }
                            case intoWarehouse2: {
                                FreightSensor.update();
                                freight = FreightSensor.getFreight();
                                switch (freight) {
                                    case CUBE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                                        break;
                                    case BALL:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                        break;
                                    case NONE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                        break;
                                }

                                if (!drive.isBusy()) {
                                    depositController.hold();
                                    drive.cancel();
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.toHub3);
//                                    redWarehousePaths.generateToHub3();
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.toHub3);
                                }
                                break;
                            }
                            case toHub3: {
                                if (!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.delivering3);
                                    depositController.set(true);
                                    deliveryTimer.reset();
                                }
                                break;
                            }
                            case delivering3: {
                                if (deliveryTimer.seconds() > .75) {
                                    depositController.set(false);
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.intoWarehouse3);
                                    drive.followTrajectorySequenceAsync(redWarehousePaths.intoWarehouse3);
                                }
                                break;
                            }
                            case intoWarehouse3: {
                                if (!drive.isBusy()) {
                                    redWarehousePaths.setPath(RedWarehousePaths.Paths.start);
                                    requestOpModeStop();
                                }
                                break;
                            }
                        }
                        break;
                    }
                    case BLUE: {
                        switch (blueWarehousePaths.path) {
                            case start: {
                                drive.setPoseEstimate(blueWarehousePaths.startingPose);
                                switch (position) {
                                    case LEFT:
                                        drive.followTrajectorySequenceAsync(blueWarehousePaths.toHubLeft);
                                        break;

                                    case CENTER:
                                        drive.followTrajectorySequenceAsync(blueWarehousePaths.toHubCenter);
                                        break;

                                    case UNKNOWN:
                                    case RIGHT:
                                        drive.followTrajectorySequenceAsync(blueWarehousePaths.toHubRight);
                                        break;
                                }
                                blueWarehousePaths.setPath(BlueWarehousePaths.Paths.toHub);
                                break;
                            }
                            case toHub: {
                                if (!drive.isBusy()) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.delivering);
                                    deliveryTimer.reset();
                                    depositController.set(true);
                                }
                                break;
                            }
                            case delivering: {
                                if (deliveryTimer.seconds() > 0) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.intoWarehouse);
                                    switch (position) {
                                        case LEFT:
                                            drive.followTrajectorySequenceAsync(blueWarehousePaths.intoWarehouseLeft);
                                            break;

                                        case CENTER:
                                            drive.followTrajectorySequenceAsync(blueWarehousePaths.intoWarehouseCenter);
                                            break;

                                        case RIGHT:
                                            drive.followTrajectorySequenceAsync(blueWarehousePaths.intoWarehouseRight);
                                            break;
                                    }
                                }
                                break;
                            }
                            case intoWarehouse: {
                                FreightSensor.update();
                                freight = FreightSensor.getFreight();
                                switch (freight) {
                                    case CUBE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                                        break;
                                    case BALL:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                        break;
                                    case NONE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                        break;
                                }

                                if (!drive.isBusy()) {
                                    drive.cancel();
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.toHub2);
//                                    blueWarehousePaths.generateToHub2();
                                    drive.followTrajectorySequenceAsync(blueWarehousePaths.toHub2);
                                }
                                break;
                            }
                            case toHub2: {
                                if (!drive.isBusy()) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.delivering2);
                                    depositController.set(true);
                                    deliveryTimer.reset();

                                }
                                break;
                            }
                            case delivering2: {
                                if (deliveryTimer.seconds() > 0) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.intoWarehouse2);
                                    drive.followTrajectorySequenceAsync(blueWarehousePaths.intoWarehouse2);
                                }
                                break;
                            }
                            case intoWarehouse2: {
                                FreightSensor.update();
                                freight = FreightSensor.getFreight();
                                switch (freight) {
                                    case CUBE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                                        break;
                                    case BALL:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                        break;
                                    case NONE:
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                        break;
                                }

                                if (!drive.isBusy()) {
                                    drive.cancel();
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.toHub3);
//                                    blueWarehousePaths.generateToHub3();
                                    drive.followTrajectorySequenceAsync(blueWarehousePaths.toHub3);
                                }
                                break;
                            }
                            case toHub3: {
                                if (!drive.isBusy()) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.delivering3);
                                    depositController.set(true);
                                    deliveryTimer.reset();
                                }
                                break;
                            }
                            case delivering3: {
                                if (deliveryTimer.seconds() > 0) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.intoWarehouse3);
                                    drive.followTrajectorySequenceAsync(blueWarehousePaths.intoWarehouse3);
                                }
                                break;
                            }
                            case intoWarehouse3: {
                                if (!drive.isBusy()) {
                                    blueWarehousePaths.setPath(BlueWarehousePaths.Paths.start);
                                    requestOpModeStop();
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                break;
            }
            case DUCK: {
                switch (PoseStorage.alliance) {
                    case RED: {
                        switch (redDuckPaths.path) {
                            case start: {
                                drive.setPoseEstimate(redDuckPaths.startingPose);
                                drive.followTrajectorySequenceAsync(redDuckPaths.toDuck);
                                redDuckPaths.setPath(RedDuckPaths.Paths.toDuck);
                                break;
                            }
                            case toDuck: {
                                if (!drive.isBusy()) {
                                    redDuckPaths.setPath(RedDuckPaths.Paths.spinningDuck);
                                    duck.spinDuckAuto();
                                }
                                break;
                            }
                            case spinningDuck: {
                                if (!duck.isBusy()) {
                                    switch (position) {
                                        case LEFT: {
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toHubLeft);
                                            break;
                                        }
                                        case CENTER: {
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toHubCenter);
                                            break;
                                        }
                                        default:
                                        case RIGHT: {
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toHubRight);
                                            break;
                                        }
                                    }
                                    redDuckPaths.setPath(RedDuckPaths.Paths.toHub);
                                }
                                break;
                            }
                            case toHub: {
                                if (!drive.isBusy()) {
                                    redDuckPaths.setPath(RedDuckPaths.Paths.delivering);
                                    depositController.set(true);
                                    deliveryTimer.reset();
                                }
                                break;
                            }
                            case delivering: {
                                if (deliveryTimer.seconds() > .75) {
                                    redDuckPaths.setPath(RedDuckPaths.Paths.toStorage);
                                    depositController.set(false);
                                    armController.setPosition((int) armStartPos);
                                    switch (position) {
                                        case LEFT:
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toStorageLeft);
                                            break;

                                        case CENTER:
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toStorageCenter);
                                            break;

                                        default:
                                        case RIGHT:
                                            drive.followTrajectorySequenceAsync(redDuckPaths.toStorageRight);
                                            break;
                                    }
                                }
                                break;
                            }
                            case toStorage: {
                                if (!drive.isBusy()) {
                                    redDuckPaths.setPath(RedDuckPaths.Paths.start);
                                    requestOpModeStop();
                                }
                                break;
                            }
                        }
                        break;
                    }
                    case BLUE: {
                        switch (blueDuckPaths.path) {
                            case start: {
                                drive.setPoseEstimate(blueDuckPaths.startingPose);
                                drive.followTrajectorySequenceAsync(blueDuckPaths.toDuck);
                                blueDuckPaths.setPath(BlueDuckPaths.Paths.toDuck);
                                break;
                            }
                            case toDuck: {
                                if (!drive.isBusy()) {
                                    blueDuckPaths.setPath(BlueDuckPaths.Paths.spinningDuck);
                                    duck.spinDuckAuto();
                                }
                                break;
                            }
                            case spinningDuck: {
                                if (!duck.isBusy()) {
                                    switch (position) {
                                        case LEFT: {
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toHubLeft);
                                            break;
                                        }
                                        case CENTER: {
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toHubCenter);
                                            break;
                                        }
                                        default:
                                        case RIGHT: {
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toHubRight);
                                            break;
                                        }
                                    }
                                    blueDuckPaths.setPath(BlueDuckPaths.Paths.toHub);
                                }
                                break;
                            }
                            case toHub: {
                                if (!drive.isBusy()) {
                                    blueDuckPaths.setPath(BlueDuckPaths.Paths.delivering);
                                    depositController.set(true);
                                    deliveryTimer.reset();
                                }
                                break;
                            }
                            case delivering: {
                                if (deliveryTimer.seconds() > .75) {
                                    blueDuckPaths.setPath(BlueDuckPaths.Paths.toStorage);
                                    depositController.set(false);
                                    armController.setPosition((int) armStartPos);
                                    switch (position) {
                                        case LEFT:
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toStorageLeft);
                                            break;

                                        case CENTER:
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toStorageCenter);
                                            break;

                                        default:
                                        case RIGHT:
                                            drive.followTrajectorySequenceAsync(blueDuckPaths.toStorageRight);
                                            break;
                                    }
                                }
                                break;
                            }
                            case toStorage: {
                                if (!drive.isBusy()) {
                                    blueDuckPaths.setPath(BlueDuckPaths.Paths.start);
                                    requestOpModeStop();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Position findPosition() {
        distances = drive.getDistLocalizer().getDistances();
        frontDist = distances[0];
        backDist = distances[1];
        leftDist = distances[2];
        rightDist = distances[3];

        PoseStorage.Alliance alliance;
        PoseStorage.StartingPosition startingPosition;

        if (leftDist > 50) {
            startingPosition = PoseStorage.StartingPosition.WAREHOUSE;
            alliance = BLUE;
        } else if (rightDist > 50) {
            startingPosition = PoseStorage.StartingPosition.WAREHOUSE;
            alliance = PoseStorage.Alliance.RED;
        } else if (leftDist > rightDist) {
            startingPosition = PoseStorage.StartingPosition.DUCK;
            alliance = BLUE;
        } else {
            startingPosition = PoseStorage.StartingPosition.DUCK;
            alliance = PoseStorage.Alliance.RED;
        }
        return new Position(alliance, startingPosition);
    }
}

class Position {

    PoseStorage.Alliance alliance;
    PoseStorage.StartingPosition position;

    Position(PoseStorage.Alliance alliance, PoseStorage.StartingPosition position) {
        this.alliance = alliance;
        this.position = position;
    }

    PoseStorage.StartingPosition getPosition() {
        return position;
    }

    PoseStorage.Alliance getAlliance() {
        return alliance;
    }
}
