package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.Controllers.ConveyorController;
import org.firstinspires.ftc.teamcode.Controllers.DepositController;
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
                         armMiddlePos = 1386 - PoseStorage.armPos,
                         armBottomPos = 1600 - PoseStorage.armPos,
                         armConveyorPos =
//                                 403 - PoseStorage.armPos, // temporary
                                 410 - PoseStorage.armPos, // with new bucket
                         armCapUpPos = 1070 - PoseStorage.armPos,
                         armCapDownPos = 1816 - PoseStorage.armPos,
                         capUpPos = .601,
                         capCappedPos = .101,
                         capDownPos = 0.25,
                         capDownDownPos = 0.165,
                         capNormalPos = 0.675,
                         fastDrivingSpeed = .9,
                         slowedDrivingSpeed = .5,
                         tiltpanMaxRange = 101,
                         tapePanVisionPos = .97,
                         tapeTiltMin = .3,
                         tapeTiltMax = .5+ 35/tiltpanMaxRange,
                         tapeTiltVisionPos = 0,
                         tapeTiltNormalPos = .4,
                         tapePanNormalPos = tapePanVisionPos;

    public final PIDFCoefficients headingControllerCoefficients = new PIDFCoefficients(.012, 0.005, 0.001, 0);

    //testing this
    public final double outputFilter = 0, outputRamp = 0;

    public final DcMotor.RunMode armMode = DcMotor.RunMode.RUN_TO_POSITION;

    public static boolean intakeOn = false;
    public static boolean intakeReverse = false;
    private static boolean firstRumble = false, secondRumble = false;
    private static boolean odoUp = false;
    private static boolean driftController = false;


    public DcMotorEx intakeMotor, duckWheelMotor, conveyorMotor;
    //duckWheel port 0 expansion hub
    //intake port 1 on expansion hub

//    public long intakeStartTime = 0;
//    public long duckWheelStartTime = 0;
    SampleMecanumDrive drive;
    TeleopHeadingDriftController controller;

    public static ArmController armController;

    ElapsedTime odoTimer = new ElapsedTime(), armServoTimer = new ElapsedTime(),
            capTimer = new ElapsedTime(), matchTime = new ElapsedTime(), modeSwap = new ElapsedTime(),
            speedSwap = new ElapsedTime(), autoFreightTimer = new ElapsedTime(),
            intakeTimer = new ElapsedTime(), duckWheelTimer = new ElapsedTime(), armTimer = new ElapsedTime();

    double driveOffset = 0, duckDirection = 1, capPos = 0;

    AnalogInput metalDetector;
    public boolean rumbling = false, matchTimeStarted = false, capDown = false, hasFreight = false, lastHadFreight = false;


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

    public FreightSensorController.Freight freight = FreightSensorController.Freight.NONE, lastFreight = FreightSensorController.Freight.NONE;

    public double drivingSpeed = fastDrivingSpeed;

    Rev2mDistanceSensor frontFloor, backFloor;

    RevColorSensorV3 freightSensor;

    LEDController led;

    public static RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;

    double frontFloorDist = 0, backFloorDist = 0;

    FreightSensorController FreightSensor;

    DuckWheelController duck;

    double maxLoopTime = 0;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime doorTimer = new ElapsedTime();

    boolean doorShutted = true;
    
    Mean loopMean = new Mean();

    boolean pathCreated = false;

    ServoImplEx tapePanServo, tapeTiltServo;

    CRServoImplEx tapeExtendServo;

    double tapeTilt = tapeTiltVisionPos, tapePan = tapePanVisionPos;

    ConveyorController conveyorController;

    public static DepositController depositController;

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
                driveOffset = 90;
                duckDirection = -1;
                break;

            case BLUE:
                driveOffset = -90;
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

        conveyorMotor = hardwareMap.get(DcMotorEx.class, "DuckWheelMotor2");
        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor.setPower(0);

        conveyorController = new ConveyorController(conveyorMotor);

        duck = new DuckWheelController(duckWheelMotor);

        if(duckDirection == -1) {
            duck.reverse();
        }

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        controller = new TeleopHeadingDriftController(.5, .05, headingControllerCoefficients, 3);
        controller.setOutputScale(.3);


        Servo armServo = hardwareMap.get(Servo.class, "armServo");
        armServo.setDirection(Servo.Direction.REVERSE);

        Servo kickerServo = hardwareMap.get(Servo.class, "kickerServo");

        depositController = new DepositController(armServo, kickerServo);

        depositController.set(false);

        armController = new ArmController(ArmHardware2021.class, hardwareMap, armMode);

        ArmController.armSetPosPower = 1;
        metalDetector = hardwareMap.get(AnalogInput.class, "metalDetector");
        frontFloor = hardwareMap.get(Rev2mDistanceSensor.class, "frontFloor");
        backFloor = hardwareMap.get(Rev2mDistanceSensor.class, "backFloor");

        freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightSensor");
        FreightSensor = new FreightSensorController(freightSensor, metalDetector);

//        kickerServo.setDirection(Servo.Direction.REVERSE);
//        capServo = hardwareMap.get(Servo.class, "capServo");
//        capServo.setDirection(Servo.Direction.REVERSE);
//        capServo.setPosition(capNormalPos);

        tapePanServo = hardwareMap.get(ServoImplEx.class, "tapePanServo");
        tapePanServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        tapePanServo.setDirection(Servo.Direction.REVERSE);
        tapePanServo.setPosition(tapePanVisionPos);

        tapeTiltServo = hardwareMap.get(ServoImplEx.class, "tapeTiltServo");
        tapeTiltServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        tapeTiltServo.scaleRange(tapeTiltMin, tapeTiltMax);
        tapeTiltServo.setPosition(tapeTiltVisionPos);

        tapeExtendServo = hardwareMap.get(CRServoImplEx.class, "tapeExtendServo");
        tapeExtendServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        tapeExtendServo.setPower(0);

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
            tapeTiltServo.setPosition(tapeTiltNormalPos);
            tapePanServo.setPosition(tapePanNormalPos);
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
//        controller.setPIDF(headingControllerCoefficients);
//        controller.setOutputFilter(outputFilter);
//        controller.setRamping(outputRamp);
//
//        armController.setPIDF(ArmHardware2021.veloPID, DcMotor.RunMode.RUN_USING_ENCODER);
//        armController.setPIDF(ArmHardware2021.positionPID, DcMotor.RunMode.RUN_TO_POSITION);

        duck.update();

        conveyorController.update();

        if(!firstRumble) {
            if(matchTime.seconds() > 120 - 45) {
                gamepad1.rumble(1, 1, 750);
                gamepad2.rumble(1, 1, 750);
                firstRumble = true;
            }
        } else if (!secondRumble) {
            if (matchTime.seconds() > 120 - 30) {
                gamepad1.rumble(.75, .75, 750);
                gamepad2.rumble(.75, .75, 750);
                secondRumble = true;
            }
        }

        controls: {

            driver: {
                if(this.gamepad1.y) {
                    driveOffset = Math.toDegrees(pose.getHeading());
                }

                if(gamepad2.share && modeSwap.seconds() > 1) {
                    modeSwap.reset();
                    gunningMode = gunningMode == GunningMode.NORMAL ? GunningMode.CAP : GunningMode.NORMAL;
                }

                if(doorTimer.seconds() > .5 && !doorShutted) {
                    depositController.set(false);
                    doorShutted = true;
                }
                if(gamepad1.a && armServoTimer.seconds() > .5) {
                    armServoTimer.reset();
                    lastHadFreight = false;
                    lastFreight = FreightSensorController.Freight.NONE;
                    freight = FreightSensorController.Freight.NONE;
                    if(armController.getSetPosition() == armStartPos) {
                        depositController.set(false);
                    } else {
                        depositController.set(!depositController.getOut());
                        if(armController.getSetPosition() == armConveyorPos) {
                            conveyorController.spin((pose.getHeading() < 0));
                            doorTimer.reset();
                            doorShutted = false;
                        }
                    }

                }

                drivetrain: {
                    if(drivingMode == DrivingMode.MANUAL) {
                        if (Math.abs(gamepad1.left_stick_x) > .05 || Math.abs(gamepad1.left_stick_y) > .05 || gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
                            Vector2d input = new Vector2d(
                                    expTranslation ? Math.pow(gamepad1.left_stick_y, driveExp) * Math.signum(-gamepad1.left_stick_y) : -gamepad1.left_stick_y,
                                    expTranslation ? Math.pow(gamepad1.left_stick_x, driveExp) * Math.signum(-gamepad1.left_stick_x) : -gamepad1.left_stick_x
                            ).rotated(-pose.getHeading() + Math.toRadians(driveOffset));
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
                            if(PoseStorage.alliance == PoseStorage.Alliance.RED) {
                                TrajectorySequence toHub;
                                if(pose.getY() < -24 && pose.getX() > 24) {
                                    drivingMode = DrivingMode.AUTO;
                                    drive.cancel();
                                    toHub = drive.trajectorySequenceBuilder(pose)
                                            .lineToLinearHeading(new Pose2d(30, -65.5, 0))
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
                            } else {
                                TrajectorySequence toHub;
                                if(pose.getY() > 24 && pose.getX() > 24) {
                                    drivingMode = DrivingMode.AUTO;
                                    drive.cancel();
                                    toHub = drive.trajectorySequenceBuilder(pose)
                                            .lineToLinearHeading(new Pose2d(30, 65.5, -0))
                                            .lineToLinearHeading(new Pose2d(-1.5, 65.5, -0))
                                            .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                                            .splineToLinearHeading(new Pose2d(-5.25, 42, Math.toRadians(-45)), Math.toRadians(-90),
                                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                                            DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                                            .build();
                                    drive.followTrajectorySequenceAsync(toHub);

                                } else if(pose.getX() < 24 && pose.getY() > 24) {
                                    drivingMode = DrivingMode.AUTO;
                                    drive.cancel();
                                    toHub = drive.trajectorySequenceBuilder(pose)
                                            .addDisplacementMarker(.2, 0, () -> TeleOPV1.armController.setPosition((int) MainAutoV1.armTopPos))
                                            .splineToLinearHeading(new Pose2d(-5.25, 42, Math.toRadians(45)), Math.toRadians(-90),
                                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .75,
                                                            DriveConstants.MAX_ANG_VEL * .75, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                                            .build();
                                    drive.followTrajectorySequenceAsync(toHub);
                            }


                        }}
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

                    if((gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_down) && armTimer.seconds() > .5){
//                        if(armMode != DcMotor.RunMode.RUN_TO_POSITION) {
//                            armMode = DcMotor.RunMode.RUN_TO_POSITION;
//                            armController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        }
                        armTimer.reset();
                        armController.setPower(ArmController.armSetPosPower);
                        if(gamepad2.dpad_up) {
                            if(armController.getSetPosition() == armTopPos){
                                armController.setPosition((int) (armStartPos));
                                depositController.set(false);
                            } else {
                                armController.setPosition((int) (armTopPos));
                                intakeOn = false;
                                depositController.hold();
                            }
                        }
                        if(gamepad2.dpad_right) {
                            if(armController.getSetPosition() == armMiddlePos){
                                armController.setPosition((int) (armStartPos));
                                depositController.set(false);

                            } else {
                                armController.setPosition((int) (armMiddlePos));
                                depositController.hold();
                            }
                            intakeOn = false;
                        }
                        if(gamepad2.dpad_left) {
                            if(armController.getSetPosition() == armConveyorPos){
                                armController.setPosition((int) (armStartPos));
                                depositController.set(false);
                            } else {
                                armController.setPosition((int) (armConveyorPos));
                                depositController.hold();
                                intakeOn = false;
                            }
                        }
                        if(gamepad2.dpad_down) {
                            if(armController.getSetPosition() == armBottomPos){
                                armController.setPosition((int) (armStartPos));
                                depositController.set(false);

                            } else {
                                armController.setPosition((int) (armBottomPos));
                                depositController.hold();
                            }
                            intakeOn = false;
                        }
                    } else if(armMode == DcMotor.RunMode.RUN_USING_ENCODER) armController.setPower(0);


//                    if(capPos != capNormalPos) {
//                        capPos = capNormalPos;
////                        capServo.setPosition(capNormalPos);
//                    }
                }
            } else
                {
                arm: {

                if(depositController.getOut()) {
                    depositController.set(false);
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
                        }
//                        if(gamepad2.dpad_down) {
//                            armController.setPosition((int) (armCapDownPos));
//                            capPos = capDownPos;
//                            capServo.setPosition(capDownPos);
//                        }
                    } else if(armMode == DcMotor.RunMode.RUN_USING_ENCODER) armController.setPower(0);

//                    if(armController.getSetPosition() == armCapUpPos) {
//                        if(gamepad2.a && capTimer.seconds() > .5) {
//                            capTimer.reset();
//                            if(capPos == capUpPos) {
//                                capPos = capCappedPos;
//                                capServo.setPosition(capCappedPos);
//                            } else {
//                                capPos = capUpPos;
//                                capServo.setPosition(capUpPos);
//                            }
//                        }
//                    } else if(armController.getSetPosition() == armStartPos) {
//                        capPos = capNormalPos;
//                        capServo.setPosition(capNormalPos);
//                    } else if(armController.getSetPosition() == armCapDownPos) {
//                        if(gamepad2.a && capTimer.seconds() > .5) {
//                            capTimer.reset();
//                            if (capPos == capDownPos) {
//                                capPos = capDownDownPos;
//                                capServo.setPosition(capDownDownPos);
//                            } else {
//                                capPos = capDownPos;
//                                capServo.setPosition(capDownPos);
//                            }
//                        }
//                    }
                }
                intakeMotor.setPower(0);
            }
//            telemetry.addData("CapPosition", capServo.getPosition());
            telemetry.addData("ArmPosition", armController.getPosition());

            tape: {
                if(gamepad2.left_trigger > .05 || gamepad2.right_trigger > .05) {
                    tapeExtendServo.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                } else tapeExtendServo.setPower(0);

                if(Math.abs(gamepad2.left_stick_y) > .05) {
                    if(Math.abs(tapeTilt - gamepad2.left_stick_y/10) < 1)
                    {
                        tapeTilt -= gamepad2.left_stick_y/10;
                        tapeTiltServo.setPosition(tapeTilt);
                    }
                }
                if(Math.abs(gamepad2.left_stick_x) > .05) {
                   if(Math.abs(tapePan + gamepad2.left_stick_x/20) < 1){
                       tapePan += gamepad2.left_stick_x/30;
                       tapePanServo.setPosition(tapePan);
                   }
                }


                telemetry.addData("tiltPos", tapeTilt);
                telemetry.addData("panPos", tapePan);


            }


                duckwheels: {
                    if(gamepad1.left_bumper && gamepad1.right_bumper) {
                        duck.slingDuck();
                    } else if(gamepad1.right_bumper && duckWheelTimer.seconds() > 2) {
                        duck.spinDuck();
                        duckWheelTimer.reset();
                    } else if (gamepad1.left_bumper && duckWheelTimer.seconds() > 2) {
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
            FreightSensor.update();
            if(armController.getPosition() < 20) {
                freight = FreightSensor.getFreight() == FreightSensorController.Freight.NONE ? freight : FreightSensor.getFreight();
            }

            if(freight == FreightSensorController.Freight.HEAVYCUBE && !(lastFreight == FreightSensorController.Freight.HEAVYCUBE)) {
                gamepad1.rumble(1.0, 1.0, 300);
                gamepad2.rumble(1.0, 1.0, 300);
            }
            lastFreight = freight;
//            freightDist = freightSensor.getDistance(DistanceUnit.INCH);
//            freightDists.push(freightDist);



            switch (freight) {
                case HEAVYCUBE:
                    depositController.hold();
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    if(!lastHadFreight) {
                        intakeReverse = true;
                        lastHadFreight = true;
                    }
                    break;
                case CUBE:
                    depositController.hold();
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    if(!lastHadFreight) {
                        intakeReverse = true;
                        lastHadFreight = true;
                    }
                    break;
                case BALL:
                    depositController.hold();
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    if(!lastHadFreight) {
                        intakeReverse = true;
                        lastHadFreight = true;
                    }
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
            frontFloorDist = frontFloor.getDistance(DistanceUnit.INCH);
            backFloorDist = backFloor.getDistance(DistanceUnit.INCH);

            double yAngle = Math.atan2(frontFloorDist-backFloorDist, 10.25);
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

    //some simple stackoverflow code useful for stuff with our scaled (tilt) servo on the tape.
    public static double scale(final double valueIn, final double baseMin, final double baseMax, final double limitMin, final double limitMax) {
        return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
    }
}

