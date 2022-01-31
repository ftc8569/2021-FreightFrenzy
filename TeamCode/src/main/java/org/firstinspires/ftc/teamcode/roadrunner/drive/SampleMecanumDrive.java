package org.firstinspires.ftc.teamcode.roadrunner.drive;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Controllers.DistanceSensorArrayLocalizer;
import org.firstinspires.ftc.teamcode.Controllers.KalmanLocalizer;
import org.firstinspires.ftc.teamcode.Controllers.MaxBoticsArray;
import org.firstinspires.ftc.teamcode.Controllers.MaxBoticsMB1040;
import org.firstinspires.ftc.teamcode.Controllers.OdoMechDistLocalizer;
import org.firstinspires.ftc.teamcode.Controllers.OdoRetractionController;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID_BACK_WHEELS;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.35;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double FOLLOWER_TIMEOUT = .25;
    public static double FOLLOWER_HEADING_TOLERANCE = Math.toRadians(.5);
    public static double FOLLOWER_POSITION_TOLERANCE = 0.25;

    public static double HEADING_LOW_PASS_CONSTANT = .75; //1 means fully the current value, 0 means fully the previous value

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu, imu1;
    private VoltageSensor batteryVoltageSensor;

    private double lastHeading, lastHeadingVelo, headingAvg, headingVeloAvg;

    private boolean lastHeadingSet, lastHeadingVeloSet;

    FtcDashboard dashboard;

    TelemetryPacket packet = new TelemetryPacket();

    TwoWheelTrackingLocalizer odolocalizer;
    MecanumLocalizer wheelLocalizer;
    DistanceSensorArrayLocalizer distLocalizer;
    KalmanLocalizer localizer;
    OdoRetractionController retractionController;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(FOLLOWER_POSITION_TOLERANCE, FOLLOWER_POSITION_TOLERANCE, FOLLOWER_HEADING_TOLERANCE), FOLLOWER_TIMEOUT);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        imu.initialize(parameters);
        imu1.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID, MOTOR_VELO_PID_BACK_WHEELS);
        }

        AnalogInput front = hardwareMap.get(AnalogInput.class, "frontSensor");
        MaxBoticsMB1040 frontSensor = new MaxBoticsMB1040(front);
        AnalogInput back = hardwareMap.get(AnalogInput.class, "backSensor");
        MaxBoticsMB1040 backSensor = new MaxBoticsMB1040(back);
        AnalogInput left = hardwareMap.get(AnalogInput.class, "leftSensor");
        MaxBoticsMB1040 leftSensor = new MaxBoticsMB1040(left);
        AnalogInput right = hardwareMap.get(AnalogInput.class, "rightSensor");
        MaxBoticsMB1040 rightSensor = new MaxBoticsMB1040(right);

        DigitalChannelImpl start = hardwareMap.get(DigitalChannelImpl.class, "start");

        MaxBoticsArray array = new MaxBoticsArray(start, front, back, left, right);

        retractionController = new OdoRetractionController(hardwareMap);

        distLocalizer = new DistanceSensorArrayLocalizer(array);
        wheelLocalizer = new MecanumLocalizer(this, true);
        odolocalizer = new TwoWheelTrackingLocalizer(hardwareMap, this);

        localizer = new KalmanLocalizer(this, odolocalizer, wheelLocalizer, distLocalizer, retractionController);


        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//        setLocalizer(new MecanumLocalizer(this, true));
        setLocalizer(localizer);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }



    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    @SuppressLint("DefaultLocale")
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);

        addTelemetry(trajectorySequenceRunner.getPacket());

        double filteredHeading = getRawExternalHeading();
        double filteredVelo = getExternalHeadingVelocity() != null ? getExternalHeadingVelocity() : 0;
//        packet.put("Original heading", headingAvg);
//        packet.put("Filtered Heading", filteredHeading);
//        packet.put("Original Velo", headingVeloAvg);
//        packet.put("Filtered Velo", filteredVelo);
//        packet.put("wheel positions", String.format("Parallel %.2f, Perpendicular %.2f", odolocalizer.getWheelPositions().get(0), odolocalizer.getWheelPositions().get(1)));
//        packet.put("Distances FBLR", Arrays.toString(localizer.getDistances()));
//        packet.put("Best Estimate", localizer.getPoseEstimate());
//        packet.put("Distance Sesnor Estimate", localizer.getDistEstimate());
//        packet.put("Mecanum Localizer Estimate", localizer.getWheelEstimate());
//        packet.put("Odometry Localizer Estimate", localizer.getOdoEstimate());
        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients frontCoefficients, PIDFCoefficients backCoefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                frontCoefficients.p, frontCoefficients.i, frontCoefficients.d,
                frontCoefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        PIDFCoefficients compensatedBackCoefficients = new PIDFCoefficients(
                backCoefficients.p, backCoefficients.i, backCoefficients.d,
                backCoefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        motors.get(0).setPIDFCoefficients(runMode, compensatedCoefficients);
        motors.get(1).setPIDFCoefficients(runMode, compensatedBackCoefficients);
        motors.get(2).setPIDFCoefficients(runMode, compensatedBackCoefficients);
        motors.get(3).setPIDFCoefficients(runMode, compensatedCoefficients);

//        for (DcMotorEx motor : motors) {
//            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
//        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {

        Orientation imu = this.imu.getAngularOrientation();
        Orientation imu1 = this.imu1.getAngularOrientation();
        headingAvg = Math.atan2(Math.sin(imu.firstAngle) + Math.sin(imu1.firstAngle), Math.cos(imu.firstAngle) + Math.cos(imu1.firstAngle));
        if(!lastHeadingSet) {
            lastHeading = headingAvg;
            lastHeadingSet = true;
        }
        double lowpassedHeading = HEADING_LOW_PASS_CONSTANT * headingAvg + (1-HEADING_LOW_PASS_CONSTANT) * lastHeading;
        lastHeading = headingAvg;
        return lowpassedHeading;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        headingVeloAvg = (imu.getAngularVelocity().zRotationRate + imu1.getAngularVelocity().zRotationRate)/2.0;
        if(!lastHeadingVeloSet) {
            lastHeadingVelo = headingVeloAvg;
            lastHeadingVeloSet = true;
        }
        double lowPassedHeadingVelo = HEADING_LOW_PASS_CONSTANT * headingVeloAvg + (1-HEADING_LOW_PASS_CONSTANT) * lastHeadingVelo;
        lastHeadingVelo = headingVeloAvg;
        return lowPassedHeadingVelo;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
    public void setPID(PIDCoefficients translation, PIDCoefficients heading) {
        follower = new HolonomicPIDVAFollower(
                translation,
                translation,
                heading,
                new Pose2d( FOLLOWER_POSITION_TOLERANCE,
                        FOLLOWER_POSITION_TOLERANCE,
                        FOLLOWER_HEADING_TOLERANCE),
                FOLLOWER_TIMEOUT
        );

    }

    public void cancel() {
        trajectorySequenceRunner.cancel();
        setMotorPowers(0,0,0,0);
    }

    public void addTelemetry(@NotNull Map<String, Object> map) {
        this.packet.putAll(map);
    }


    public void setOdometry(boolean up) {
        retractionController.set(up);
    }


    public KalmanLocalizer getCurrentLocalizer() {
        return localizer;
    }

    public TwoWheelTrackingLocalizer getOdolocalizer() {
        return odolocalizer;
    }

    public MecanumLocalizer getWheelLocalizer() {
        return wheelLocalizer;
    }

    public DistanceSensorArrayLocalizer getDistLocalizer() {
        return distLocalizer;
    }
}
