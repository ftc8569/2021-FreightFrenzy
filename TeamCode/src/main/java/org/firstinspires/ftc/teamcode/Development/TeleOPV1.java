package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.Controllers.DuckWheelController;
import org.firstinspires.ftc.teamcode.Controllers.FreightSensorController;
import org.firstinspires.ftc.teamcode.Controllers.LEDController;
import org.firstinspires.ftc.teamcode.Controllers.TeleopHeadingDriftController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.HashMap;

@Config
@TeleOp(name = "TeleOPV1", group = "Development")

public class TeleOPV1 extends OpMode {

    public final double driveExp = 2; // The power to raise all driving movements to. Specifically
    public final boolean expTranslation = true; //useful for turning but can definitely help with translation as well

    public static double intakeSpeed = 1,
                         intakeOutSpeed = .4,
                         duckWheelSpeed = .6,
                         armStartPos = 0 - PoseStorage.armPos,
                         armTopPos = 1070 - PoseStorage.armPos,
                         armMiddlePos = 1466 - PoseStorage.armPos,
                         armBottomPos = 1600 - PoseStorage.armPos,
                         armServoShutPos = 0,
                         armServoOpenPos = .1,
                         armCapUpPos = 1070 - PoseStorage.armPos,
                         armCapDownPos = 1816 - PoseStorage.armPos,
                         capUpPos = .601,
                         capCappedPos = .101,
                         capDownPos = 0.25,
                         capDownDownPos = 0.165,
                         capNormalPos = 0.675,
                         fastDrivingSpeed = .9,
                         slowedDrivingSpeed = .5;

    public final PIDFCoefficients headingControllerCoefficients = new PIDFCoefficients(.012, 0.005, 0.001, 0);

    //testing this
    public final double outputFilter = 0, outputRamp = 0;

    public final DcMotor.RunMode armMode = DcMotor.RunMode.RUN_TO_POSITION;

    public static boolean intakeOn = false;
    public static boolean intakeReverse = false;
    private static boolean firstRumble = false, secondRumble = false;
    private static boolean odoUp = false, armServoShut = true;
    private static final boolean driftController = false;


    public DcMotorEx intakeMotor, duckWheelMotor, duckWheelMotor2;
    //duckWheel port 0 expansion hub
    //intake port 1 on expansion hub

//    public long intakeStartTime = 0;
//    public long duckWheelStartTime = 0;
    SampleMecanumDrive drive;
    TeleopHeadingDriftController controller;

    public static ArmController armController;

    Servo armServo, capServo;

    ElapsedTime odoTimer = new ElapsedTime(), armServoTimer = new ElapsedTime(),
            capTimer = new ElapsedTime(), matchTime = new ElapsedTime(), modeSwap = new ElapsedTime(),
            speedSwap = new ElapsedTime(), autoFreightTimer = new ElapsedTime(),
            intakeTimer = new ElapsedTime(), duckWheelTimer = new ElapsedTime();

    double driveOffset = 0, duckDirection = 1, capPos = 0;

//    AnalogInput metalDetector;
    public static boolean rumbling = false, matchTimeStarted = false, hasmetal = false, lastHadMetal = false, capDown = false, hasFreight = false;


    public enum GunningMode {
        NORMAL,
        CAP
    }

    public GunningMode gunningMode = GunningMode.NORMAL;

    public enum DrivingMode {
        MANUAL,
        AUTO
    }

    public DrivingMode drivingMode = DrivingMode.MANUAL;

    public FreightSensorController.Freight freight = FreightSensorController.Freight.NONE;

    public double drivingSpeed = fastDrivingSpeed;

//    Rev2mDistanceSensor frontFloor, backFloor;

    RevColorSensorV3 freightSensor;

    LEDController led;

    double frontFloorDist = 0, backFloorDist = 0;

    FreightSensorController FreightSensor;

    DuckWheelController duck;

    double maxLoopTime = 0;

    ElapsedTime timer = new ElapsedTime();
    
    Mean loopMean = new Mean();

    boolean pathCreated = false;

    @Override
    public void init() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        led = new LEDController(hardwareMap.get(RevBlinkinLedDriver.class, "led"));
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

        gunningMode = GunningMode.NORMAL;
        drivingMode = DrivingMode.MANUAL;



        switch (PoseStorage.alliance) {
            case RED:
                driveOffset = -90;
                duckDirection = -1;
                break;

            case BLUE:
                driveOffset = 90;
                duckDirection = 1;
                break;

            case NEITHER:
                driveOffset = 0;
                duckDirection = -1;
                break;
        }

        DriveConstants.MAX_VEL *= fastDrivingSpeed;
        DriveConstants.MAX_ACCEL *= fastDrivingSpeed;
        DriveConstants.MAX_ANG_VEL *= fastDrivingSpeed;
        DriveConstants.MAX_ANG_ACCEL *= fastDrivingSpeed;
        drive = new SampleMecanumDrive(hardwareMap);

        duckWheelMotor = hardwareMap.get(DcMotorEx.class, "duckWheelMotor");
        duckWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheelMotor.setPower(0);

        duckWheelMotor2 = hardwareMap.get(DcMotorEx.class, "DuckWheelMotor2");
        duckWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckWheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheelMotor2.setPower(0);

        duck = new DuckWheelController(duckWheelMotor, duckWheelMotor2);

        if(duckDirection == -1) {
            duck.reverse();
        }

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        controller = new TeleopHeadingDriftController(.5, .05, headingControllerCoefficients, 3);
        controller.setOutputScale(.3);


        armServo = hardwareMap.get(Servo.class, "armServo");
        armServo.setPosition(armServoShutPos);

        armController = new ArmController(ArmHardware2021.class, hardwareMap, armMode);

        ArmController.armSetPosPower = 1;
//        metalDetector = hardwareMap.get(AnalogInput.class, "metalDetector");
//        frontFloor = hardwareMap.get(Rev2mDistanceSensor.class, "frontFloor");
//        backFloor = hardwareMap.get(Rev2mDistanceSensor.class, "backFloor");

        freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightSensor");
        FreightSensor = new FreightSensorController(freightSensor);

        capServo = hardwareMap.get(Servo.class, "capServo");
        capServo.setDirection(Servo.Direction.REVERSE);
        capServo.setPosition(capNormalPos);

        telemetry.addData(">", "Initialized!!!");
        telemetry.update();
        while(led.getSeconds() < 1) {}
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    @Override
    public void loop() {
        if(!matchTimeStarted) {
            matchTime.reset();
            timer.reset();
            matchTimeStarted = true;
        } else {
            double dt = timer.milliseconds();
            loopMean.increment(dt);
            if(dt > maxLoopTime) maxLoopTime = dt;
            timer.reset();
            telemetry.addData("avg Loop Time",loopMean.getResult());
            telemetry.addData("max loop time", maxLoopTime);
        }
        if(PoseStorage.endPose != null) {
            drive.setPoseEstimate(PoseStorage.endPose);
            PoseStorage.endPose = null;
        }
        Pose2d power = new Pose2d(0, 0, 0);
        drive.update();
        Pose2d pose = drive.getPoseEstimate();
        long millis = System.currentTimeMillis();
//        controller.setPIDF(headingControllerCoefficients);
//        controller.setOutputFilter(outputFilter);
//        controller.setRamping(outputRamp);
//
//        armController.setPIDF(ArmHardware2021.veloPID, DcMotor.RunMode.RUN_USING_ENCODER);
//        armController.setPIDF(ArmHardware2021.positionPID, DcMotor.RunMode.RUN_TO_POSITION);

        duck.update();

        if(!firstRumble) {
            if(matchTime.seconds() > 120 - 35) {
                gamepad1.rumble(1, 1, 750);
                gamepad2.rumble(1, 1, 750);
                firstRumble = true;
            }
        } else if (!secondRumble) {
            if (matchTime.seconds() > 120 - 30) {
                gamepad1.rumble(.5, .5, 500);
                gamepad2.rumble(.5, .5, 500);
                secondRumble = true;
            }
        }

        controls: {

            driver: {
                if(this.gamepad1.y) {
                    drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),0));
                    controller.reset(drive.getPoseEstimate());
                }

                if(gamepad2.share && modeSwap.seconds() > 1) {
                    modeSwap.reset();
                    gunningMode = gunningMode == GunningMode.NORMAL ? GunningMode.CAP : GunningMode.NORMAL;
                }

                if(gamepad1.a && armServoTimer.seconds() > .5) {
                    armServo.setPosition(armServoShut ? armServoOpenPos : armServoShutPos);
                    armServoShut = !armServoShut;
                    armServoTimer.reset();
                    freight = FreightSensorController.Freight.NONE;
                }

                drivetrain: {
                    if(drivingMode == DrivingMode.MANUAL) {
                        if (Math.abs(gamepad1.left_stick_x) > .05 || Math.abs(gamepad1.left_stick_y) > .05 || gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
                            Vector2d input = new Vector2d(
                                    expTranslation ? Math.pow(gamepad1.left_stick_y, driveExp) * Math.signum(-gamepad1.left_stick_y) : -gamepad1.left_stick_y,
                                    expTranslation ? Math.pow(gamepad1.left_stick_x, driveExp) * Math.signum(-gamepad1.left_stick_x) : -gamepad1.left_stick_x
                            ).rotated(-pose.getHeading() + Math.toDegrees(driveOffset));
                            double rotation = Math.pow(gamepad1.left_trigger, driveExp) - Math.pow(gamepad1.right_trigger, driveExp);
                            power = new Pose2d(input.getX(), input.getY(), rotation);
//                        telemetry.addData("Input Power", power.toString());
                        } else {
                            power = new Pose2d(0, 0, 0);
//                        telemetry.addData("Input Power", new Pose2d(0,0,0).toString());
                        }
                        Pose2d controlled = controller.control(pose, power).times(drivingSpeed);
                        drive.setWeightedDrivePower(controlled);

                        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                            TrajectorySequence toHub;
                            if(pose.getY() < -24 && pose.getX() > 24) {
                                drivingMode = DrivingMode.AUTO;
                                drive.cancel();
                                toHub = drive.trajectorySequenceBuilder(pose)
                                        .lineToLinearHeading(new Pose2d(-1.5, -65.5, 0))
                                        .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                                    .splineToLinearHeading(new Pose2d(-5.25, -42, Math.toRadians(-45)), Math.toRadians(90),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                                    DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                                    .build();
                                drive.followTrajectorySequenceAsync(toHub);

                            } else if(pose.getX() < 24 && pose.getY() < -24) {
                                drivingMode = DrivingMode.AUTO;
                                drive.cancel();
                                toHub = drive.trajectorySequenceBuilder(pose)
                                        .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                                        .splineToLinearHeading(new Pose2d(-5.25, -42, Math.toRadians(-45)), Math.toRadians(90),
                                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                                        DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                                        .build();
                                drive.followTrajectorySequenceAsync(toHub);
                            }

                        }
                    } else {
                        if(Math.abs(gamepad1.left_stick_x) > .05 || Math.abs(gamepad1.left_stick_y) > .05 || gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
                            drivingMode = DrivingMode.MANUAL;
                            drive.cancel();
                            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        }
                    }

//                    telemetry.addData("Setpoint, Target Pose, Actual, Error",
//                            Math.round(controller.getSetpoint() * 100)/100 + ", " +
//                                    Math.round(controller.getTargetPose() * 100)/100 + ", " +
//                                    Math.round(Math.toDegrees(pose.getHeading()) * 100)/100 + ", " +
//                                    Math.round(controller.getError() * 100)/100);
//                    telemetry.addData("Controlled Power", "(%.3f, %.3f, %.3f°)", controlled.getX(), controlled.getY(), controlled.getHeading());

//        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.right_stick_x) > .05) {
//            double desAng = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
//            telemetry.addData("Desired Angle deg", Math.toDegrees(desAng));
//            controller.setTargetPose(desAng);
//        }
                    if(gamepad1.x && speedSwap.seconds() > .5) {
                        drivingSpeed = drivingSpeed == fastDrivingSpeed ? slowedDrivingSpeed : fastDrivingSpeed;
                        speedSwap.reset();
                    }
                }

                odometry: {
                    if(gamepad1.b && odoTimer.seconds() > .5) {
                        odoTimer.reset();
                        if(odoUp) {
                            drive.setOdometry(false);
                            odoUp = false;
                        } else {
                            drive.setOdometry(true);
                            odoUp = true;
                        }
                    }
                }
            }

            gunner: {

            if(gunningMode == GunningMode.NORMAL) {
                intake: {
                    if(gamepad2.a && intakeTimer.seconds() > .5) {
                        intakeTimer.reset();
                        if(intakeOn) {
                            if(intakeReverse) {
                                intakeReverse = false;
                            } else intakeOn = false;
                        } else {
                            intakeOn = true;
                            intakeReverse = false;
                        }
                    } else if(gamepad2.b && intakeTimer.seconds() > .5) {
                        intakeTimer.reset();
                        if(intakeOn) {
                            if(!intakeReverse) {
                                intakeReverse = true;
                            } else intakeOn = false;
                        } else {
                            intakeOn = true;
                            intakeReverse = true;
                        }
                    }

                    if(intakeOn) intakeMotor.setPower(intakeReverse ? -intakeOutSpeed: intakeSpeed); else intakeMotor.setPower(0);


                }
                arm: {

                ArmController.armSetPosPower = 1;
//                if (Math.abs(gamepad1.right_stick_y) > .05) {
//                    if(armMode != DcMotor.RunMode.RUN_USING_ENCODER) {
//                        armMode = DcMotor.RunMode.RUN_USING_ENCODER;
//                        armController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    }
//                    armController.setPower(-gamepad1.right_stick_y);
//                } else

                    if(gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_down){
//                        if(armMode != DcMotor.RunMode.RUN_TO_POSITION) {
//                            armMode = DcMotor.RunMode.RUN_TO_POSITION;
//                            armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        }
                        armController.setPower(ArmController.armSetPosPower);
                        if(gamepad2.dpad_up) {
                            armController.setPosition((int) (armTopPos - PoseStorage.armPos));
                            intakeOn = false;
                        }
                        if(gamepad2.dpad_left || gamepad2.dpad_right) {
                            armController.setPosition((int) (armStartPos- PoseStorage.armPos));
                            armServo.setPosition(armServoShutPos);
                        }
                        if(gamepad2.dpad_down) {
                            armController.setPosition((int) (armBottomPos- PoseStorage.armPos));
                            intakeOn = false;
                        }
                    } else if(armMode == DcMotor.RunMode.RUN_USING_ENCODER) armController.setPower(0);


                    if(capPos != capNormalPos) {
                        capPos = capNormalPos;
                        capServo.setPosition(capNormalPos);
                    }
                }
            } else {
                arm: {

                if(!armServoShut) {
                    armServo.setPosition(armServoShutPos);
                    armServoShut = true;
                }
                ArmController.armSetPosPower = .75;
//                if (Math.abs(gamepad1.right_stick_y) > .05) {
//                    if(armMode != DcMotor.RunMode.RUN_USING_ENCODER) {
//                        armMode = DcMotor.RunMode.RUN_USING_ENCODER;
//                        armController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    }
//                    armController.setPower(-gamepad1.right_stick_y);
//                } else
                    if(gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_down){
//                        if(armMode != DcMotor.RunMode.RUN_TO_POSITION) {
//                            armMode = DcMotor.RunMode.RUN_TO_POSITION;
//                            armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        }
                        armController.setPower(ArmController.armSetPosPower);
                        if(gamepad2.dpad_up) {
                            armController.setPosition((int) (armCapUpPos));
//                            capPos = capUpPos;
//                            capServo.setPosition(capUpPos);
                        }
                        if(gamepad2.dpad_left || gamepad2.dpad_right) {
                            armController.setPosition((int) (armStartPos));
                            armServo.setPosition(armServoShutPos);
                        }
                        if(gamepad2.dpad_down) {
                            armController.setPosition((int) (armCapDownPos));
                            capPos = capDownPos;
                            capServo.setPosition(capDownPos);
                        }
                    } else if(armMode == DcMotor.RunMode.RUN_USING_ENCODER) armController.setPower(0);

                    if(armController.getSetPosition() == armCapUpPos) {
                        if(gamepad2.a && capTimer.seconds() > .5) {
                            capTimer.reset();
                            if(capPos == capUpPos) {
                                capPos = capCappedPos;
                                capServo.setPosition(capCappedPos);
                            } else {
                                capPos = capUpPos;
                                capServo.setPosition(capUpPos);
                            }
                        }
                    } else if(armController.getSetPosition() == armStartPos) {
                        capPos = capNormalPos;
                        capServo.setPosition(capNormalPos);
                    } else if(armController.getSetPosition() == armCapDownPos) {
                        if(gamepad2.a && capTimer.seconds() > .5) {
                            capTimer.reset();
                            if (capPos == capDownPos) {
                                capPos = capDownDownPos;
                                capServo.setPosition(capDownDownPos);
                            } else {
                                capPos = capDownPos;
                                capServo.setPosition(capDownPos);
                            }
                        }
                    }
                }
                intakeMotor.setPower(0);
            }
            telemetry.addData("CapPosition", capServo.getPosition());
            telemetry.addData("ArmPosition", armController.getPosition());



                duckwheels: {
                    if(gamepad2.right_bumper && duckWheelTimer.seconds() > 2) {
                        duck.spinDuck();
                        duckWheelTimer.reset();
                    } else if (gamepad2.left_bumper && duckWheelTimer.seconds() > 2) {
                        duck.spinDuckSlow();
                        duckWheelTimer.reset();
                    }
                }
            }
        }




        if(controller.getEnabled() != driftController) {
            controller.setEnabled(driftController);
        }

//        drive.setWeightedDrivePower(power);

        HashMap<String, Object> map = new HashMap<>();
        map.put("Setpoint", Math.toRadians(controller.getTargetPose()));
        map.put("error", Math.toRadians(controller.getError()));
        map.put("lastOutput", controller.getLastOutput());
        double[] avgs = controller.getBufferAvgs();
        map.put("BufferVal avg", avgs[0]);
        map.put("BufferTime avg", avgs[1]);
//        map.put("MetalDetectorVoltage", metalDetector.getVoltage());
//        drive.addTelemetry(map);

//        telemetry.addData("MetalDetectorVoltage", metalDetector.getVoltage());
        telemetry.addData("Pose Velocity", drive.getCurrentLocalizer().getPoseVelocity());
//       if(pipeline.BoundingRectangle != null) {
//           telemetry.addData("Rect X, Y", pipeline.BoundingRectangle.x + ", " + pipeline.BoundingRectangle.y);
//       } else  telemetry.addData("Rect X, Y", "null");







        sensors: {
//            hasmetal = metalDetector.getVoltage() > 1.5;
//            if(hasmetal && !lastHadMetal) {
//                gamepad1.rumble(1.0, 1.0, 1);
//            }
//            lastHadMetal = hasmetal;
//            freightDist = freightSensor.getDistance(DistanceUnit.INCH);
//            freightDists.push(freightDist);
            FreightSensor.update();
            freight = FreightSensor.getFreight() == FreightSensorController.Freight.NONE ? freight : FreightSensor.getFreight();
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
            // bucket: 2375, 1062, 1111, 1512 (.58 dist)
            // bucketUp: 60, 69, 63, 63 (1.65+ dist)
            //cubeFar: 175, 252, 137
            //cubeNear: 1321, 1683, 500
            //ball: 93, 156, 128, 122
//            telemetry.addData("ferightDist", freightDist);
//            telemetry.addData("ferightDists", freightDists.avg());
            telemetry.addData("freightRGBA", "%s, %s, %s, %s", FreightSensor.red, FreightSensor.green, FreightSensor.blue, FreightSensor.alpha);
            telemetry.addData("combined Freight", FreightSensor.getSum());
//            frontFloorDist = frontFloor.getDistance(DistanceUnit.INCH);
//            backFloorDist = backFloor.getDistance(DistanceUnit.INCH);

//            double yAngle = Math.atan2(frontFloorDist-backFloorDist, 10.25);
//            telemetry.addData("robotangle", Math.toDegrees(yAngle));
        }





        telemetry.addData("Distances FBLR", Arrays.toString(drive.getCurrentLocalizer().getDistances()));
//        telemetry.addData("Best Localizer", drive.getCurrentLocalizer().getBestCurrentLocalizer());
        telemetry.addData("distance Heading", Math.toDegrees(drive.getCurrentLocalizer().heading));
        telemetry.addData("Best Estimate", drive.getCurrentLocalizer().getPoseEstimate());
        telemetry.addData("Distance Sesnor Estimate", drive.getCurrentLocalizer().getDistEstimate());
        telemetry.addData("Mecanum Localizer Estimate", drive.getCurrentLocalizer().getWheelEstimate());
        telemetry.addData("Odometry Localizer Estimate", drive.getCurrentLocalizer().getOdoEstimate());


    }
}

