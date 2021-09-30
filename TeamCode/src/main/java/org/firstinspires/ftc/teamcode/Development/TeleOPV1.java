package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "TeleopV1", group = "Development")
public class TeleOPV1 extends OpMode {

    SampleMecanumDrive drive;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        Pose2d power = new Pose2d();
        drive.update();

        if(Math.abs(gamepad1.left_stick_x) > .05 || Math.abs(gamepad1.left_stick_y) > .05 || gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-drive.getExternalHeading());
            double rotation = gamepad1.left_trigger - gamepad1.right_trigger;
            power = new Pose2d(input, rotation);
        }
        drive.setWeightedDrivePower(power);
    }
}
