package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware;
import org.firstinspires.ftc.teamcode.Controllers.ArmHardware2021;
import org.firstinspires.ftc.teamcode.Controllers.MiniPID;
import org.firstinspires.ftc.teamcode.Controllers.TeleopHeadingDriftController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "TeleopV1", group = "Development")

public class TeleOPV1 extends OpMode {

    public static double driveExp = 2; // The power to raise all driving movements to. Specifically
    public static boolean expTranslation = true; //useful for turning but can definitely help with translation as well

    public static double intakeSpeed = 1,
                         duckWheelSpeed = .75;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(.01, 0, 0, 0);
    //testing this
    public static double outputFilter = 0.1;

    private static boolean intakeOn = false;
    private static boolean duckWheelOn = false;
    private static boolean intakeReverse = false;
    private static boolean rumbled = false;
    private static final boolean driftController = true;

    public DcMotorEx intakeMotor, duckWheelMotor;
    //duckWheel port 0 expansion hub
    //intake port 1 on expansion hub

    public long intakeStartTime = 0;
    public long duckWheelStartTime = 0;
    SampleMecanumDrive drive;
    TeleopHeadingDriftController controller;

    ArmController armController;


    @Override
    public void init() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        duckWheelMotor = hardwareMap.get(DcMotorEx.class, "duckWheelMotor");
        duckWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckWheelMotor.setPower(0);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        controller = new TeleopHeadingDriftController(3.5, .125, pidfCoefficients);
        controller.setOutputScale(.5);

        armController = new ArmController(ArmHardware2021.class, hardwareMap);

        telemetry.addData(">", "Initialized!!!");
        telemetry.update();
    }

    @Override
    public void loop() {
        Pose2d power = new Pose2d(0, 0, 0);
        drive.update();
        Pose2d pose = drive.getPoseEstimate();
        long millis = System.currentTimeMillis();
        controller.setPIDF(pidfCoefficients);
        controller.setOutputFilter(outputFilter);

        if(this.gamepad1.y) {
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),0));
            controller.reset(drive.getPoseEstimate());
        }

        if(controller.getEnabled() != driftController) {
            controller.setEnabled(driftController);
        }
        if(Math.abs(gamepad1.left_stick_x) > .05 || Math.abs(gamepad1.left_stick_y) > .05 || gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            Vector2d input = new Vector2d(
                    expTranslation ? Math.pow(gamepad1.left_stick_y,driveExp) * Math.signum(-gamepad1.left_stick_y):-gamepad1.left_stick_y,
                    expTranslation ? Math.pow(gamepad1.left_stick_x,driveExp) * Math.signum(-gamepad1.left_stick_x):-gamepad1.left_stick_x
            ).rotated(-pose.getHeading());
            double rotation = Math.pow(gamepad1.left_trigger,driveExp) - Math.pow(gamepad1.right_trigger,driveExp);
            power = new Pose2d(input.getX(),input.getY(), rotation);
            telemetry.addData("Input Power", power.toString());
        } else {
            power = new Pose2d(0,0,0);
            telemetry.addData("Input Power", new Pose2d(0,0,0).toString());
        }
        Pose2d controlled = controller.control(pose, power);
        drive.setWeightedDrivePower(controlled);
        telemetry.addData("Setpoint, Target Pose, Actual, Error",
          Math.round(controller.getSetpoint() * 100)/100 + ", " +
                  Math.round(controller.getTargetPose() * 100)/100 + ", " +
                  Math.round(Math.toDegrees(pose.getHeading()) * 100)/100 + ", " +
                  Math.round(controller.getError() * 100)/100);
        telemetry.addData("Controlled Power", "(%.3f, %.3f, %.3f°)", controlled.getX(), controlled.getY(), controlled.getHeading());
//        drive.setWeightedDrivePower(power);

        if(gamepad1.right_bumper && millis - intakeStartTime > 500) {
            intakeOn = !intakeOn;
            intakeStartTime = millis;
        }
        if(gamepad1.left_bumper && millis - duckWheelStartTime > 500) {
            duckWheelOn = ! duckWheelOn;
            duckWheelStartTime = millis;
        }

        if(Math.abs(gamepad1.right_stick_y) > .05) {
            armController.setPower(-gamepad1.right_stick_y);
        } else armController.setPower(0);

        if(gamepad1.b) intakeReverse = !intakeReverse;

        if(intakeOn) intakeMotor.setPower(intakeReverse ? -intakeSpeed : intakeSpeed); else intakeMotor.setPower(0);
        if(duckWheelOn) duckWheelMotor.setPower(duckWheelSpeed); else duckWheelMotor.setPower(0);
        if(gamepad1.a && !rumbled) {
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            rumbled = true;
        }

        telemetry.addData("x, y, degrees:",pose.toString());
        telemetry.addData("power", power);
    }
}

