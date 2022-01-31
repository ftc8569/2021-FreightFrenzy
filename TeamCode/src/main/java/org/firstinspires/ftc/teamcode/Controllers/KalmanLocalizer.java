package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;


public class KalmanLocalizer implements Localizer {

    SampleMecanumDrive drive;
    TwoTrackingWheelLocalizer odoLocalizer;
    MecanumDrive.MecanumLocalizer wheelLocalizer;
    DistanceSensorArrayLocalizer distLocalizer;
    OdoRetractionController odoRetractionController;

    AdjustableKalmanFilter filter;

    Pose2d bestEstimate = new Pose2d();

    ElapsedTime timer;

    double deltaTime = 0;

    boolean init = false;

    RealMatrix stateTransition = new Array2DRowRealMatrix(new double[][] {
            {1, deltaTime, 0.5*Math.pow(deltaTime,2), 0, 0, 0, 0, 0, 0},
            {0, 1, deltaTime, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, deltaTime, 0.5*Math.pow(deltaTime,2), 0, 0, 0},
            {0, 0, 0, 0, 1, deltaTime, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, deltaTime, 0.5*Math.pow(deltaTime,2)},
            {0, 0, 0, 0, 0, 0, 0, 1, deltaTime},
            {0, 0, 0, 0, 0, 0, 0, 0, 1},
    });

    RealMatrix controlMatrix = null;

    //TODO: FIGURE THIS OUT A LITTLE BETTER
    //guesstimate of the std dev of our acceleration model in in/s^2
    double processUncertainty = 5;

    RealMatrix processNoise = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0, 0, 0, 0},
            {Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0,0, 0, 0},
            {0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0},
            {0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0},
            {0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2},
            {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime},
            {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1}
    }).scalarMultiply(processUncertainty);

    RealVector initialState = new ArrayRealVector(new double[] {
            0, 0, 0, 0, 0, 0, 0, 0, 0
    });

    //TODO: this probably shouldn't be 0 but we do know pretty exactly our start pos.
    // We know we are not moving though

    //Although, we know we are not moving at first, but what if we reset our position later?
    // Our position will be accurate but we don't know if our velocity will be accurate so we
    // might add some error there for our later positons.
    double initialStateError = 0;

    double resetStateError = 10;

    RealMatrix initialStateCovarError = new Array2DRowRealMatrix(new double[][] {
            {initialStateError, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, initialStateError, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, initialStateError, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0}
    });

    RealMatrix resetStateCovarError = new Array2DRowRealMatrix(new double[][] {
            {initialStateError, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, resetStateError, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, resetStateError, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, initialStateError, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, resetStateError, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, resetStateError, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, initialStateError, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, resetStateError, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, resetStateError}
    });

    AdjustableProcessModel pm;

    RealMatrix measurementMatrix = new Array2DRowRealMatrix(new double[][]{
            {1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1, 0}
    });

    //we will currently assume our velocity uncertainty is the same as our position.
    // This is not true but I do not think we will pass it velocity - the localizer is weird.
    double measUncX = 0, measUncY = 0, measUncTheta = 0;

    //TODO: do some bench top measurements on our uncertainty here. A bit hard to do but we can get the idea.
    RealMatrix measurementUncertaintyMatrix = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(measUncX, 2), 0, 0, 0, 0, 0},
            {0, Math.pow(measUncY, 2), 0, 0, 0, 0},
            {0, 0, Math.pow(measUncTheta, 2), 0, 0, 0},
            {0, 0, 0, Math.pow(measUncX, 2), 0, 0},
            {0, 0, 0, 0, Math.pow(measUncY, 2), 0},
            {0, 0, 0, 0, 0, Math.pow(measUncTheta, 2)},
    });

    AdjustableMeasurementModel mm;

    Pose2d odoEstimate = new Pose2d(), wheelEstimate = new Pose2d(), distEstimate = new Pose2d(),
        odoVeloEstimate = new Pose2d(),wheelVeloEstimate = new Pose2d();

    double[] stateEstimate = new double[]{};

    //Vectors to give to correct()

    //TODO: maybe make a variable to control whether or not to give velo information
    //to stop giving velo information, swap the top line for the bottom commented one for right now
    RealVector odoMeasureVector = new ArrayRealVector(new double[]{
            odoEstimate.getX(), odoVeloEstimate.getX(), odoEstimate.getY(), odoVeloEstimate.getY(), odoEstimate.getHeading(), odoVeloEstimate.getHeading()
//          odoEstimate.getX(), 0, odoEstimate.getY(), 0, odoEstimate.getHeading(), 0
    });

    //to stop giving pose information, swap the last 3 values for 0s
    RealVector wheelMeasureVector = new ArrayRealVector(new double[]{
            wheelEstimate.getX(), wheelVeloEstimate.getX(), wheelEstimate.getY(), wheelVeloEstimate.getY(), wheelEstimate.getHeading(), wheelVeloEstimate.getHeading()
//          wheelEstimate.getX(), 0, wheelEstimate.getY(), 0, wheelEstimate.getHeading(), 0
    });

    RealVector distMeasureVector = new ArrayRealVector(new Double[]{
            distEstimate.getX(), 0.0, distEstimate.getY(), 0.0, distEstimate.getHeading(), 0.0
    });

    public KalmanLocalizer(@NotNull SampleMecanumDrive drive, @NotNull TwoWheelTrackingLocalizer odoLocalizer,
                           @NotNull MecanumDrive.MecanumLocalizer wheelLocalizer,
                           @NotNull DistanceSensorArrayLocalizer distLocalizer, @NotNull OdoRetractionController odoRetractionController) {
        this.drive = drive;
        this.odoLocalizer = odoLocalizer;
        this.wheelLocalizer = wheelLocalizer;
        this.distLocalizer = distLocalizer;
        this.odoRetractionController = odoRetractionController;

        timer = new ElapsedTime();

        updateFilter();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return bestEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        //remake the filter so that we can reset the initial estimate and estimate error
        initialState = new ArrayRealVector(new double[]{
                pose2d.getX(), 0, 0, pose2d.getY(), 0, 0, pose2d.getHeading(), 0, 0
        });
        updateFilter();
        wheelLocalizer.setPoseEstimate(pose2d);
        odoLocalizer.setPoseEstimate(pose2d);
        distLocalizer.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(stateEstimate[1], stateEstimate[4], stateEstimate[7]);
    }

    @Override
    public void update() {
        if(!init) {
            deltaTime = 0;
            init = true;
        } else deltaTime = timer.seconds();

        odoEstimate = odoRetractionController.isUp() ? new Pose2d() : odoLocalizer.getPoseEstimate();
        odoVeloEstimate = odoRetractionController.isUp() ? new Pose2d() : odoLocalizer.getPoseVelocity();

        wheelEstimate = wheelLocalizer.getPoseEstimate();
        wheelVeloEstimate = wheelLocalizer.getPoseVelocity();

        distLocalizer.setPoseEstimate(bestEstimate);
        distLocalizer.setPoseVelocity(Objects.requireNonNull(getPoseVelocity()));
        distEstimate = distLocalizer.getPoseEstimate();

        filter.predict();

        updateVectors();

        filter.correct(wheelMeasureVector);

        if(!odoEstimate.equals(new Pose2d())) {
            filter.correct(odoMeasureVector);
        }

        if(!distEstimate.equals(new Pose2d())) {
            filter.correct(distMeasureVector);
        }

        timer.reset();
        stateEstimate = filter.getStateEstimation();
        bestEstimate = new Pose2d(stateEstimate[0], stateEstimate[3], stateEstimate[6]);
    }

    private void updateFilter() {
        pm = new AdjustableProcessModel(stateTransition, controlMatrix, processNoise,  initialState, resetStateCovarError);
        mm = new AdjustableMeasurementModel(measurementMatrix, measurementUncertaintyMatrix);
        filter = new AdjustableKalmanFilter(pm, mm);
    }

    //TODO: do this better. DoubleSuppliers?
    private void updateVectors() {
        odoMeasureVector = new ArrayRealVector(new double[]{
                odoEstimate.getX(), odoVeloEstimate.getX(), odoEstimate.getY(), odoVeloEstimate.getY(), odoEstimate.getHeading(), odoVeloEstimate.getHeading()
//          odoEstimate.getX(), 0, odoEstimate.getY(), 0, odoEstimate.getHeading(), 0
        });

        wheelMeasureVector = new ArrayRealVector(new double[]{
                wheelEstimate.getX(), wheelVeloEstimate.getX(), wheelEstimate.getY(), wheelVeloEstimate.getY(), wheelEstimate.getHeading(), wheelVeloEstimate.getHeading()
//          wheelEstimate.getX(), 0, wheelEstimate.getY(), 0, wheelEstimate.getHeading(), 0
        });
        distMeasureVector = new ArrayRealVector(new Double[]{
                distEstimate.getX(), 0.0, distEstimate.getY(), 0.0, distEstimate.getHeading(), 0.0
        });

        //we have to recalculate our matrices that are dependant on deltaT every loop and put them into our filter.
        stateTransition = new Array2DRowRealMatrix(new double[][] {
                {1, deltaTime, 0.5*Math.pow(deltaTime,2), 0, 0, 0, 0, 0, 0},
                {0, 1, deltaTime, 0, 0, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 1, deltaTime, 0.5*Math.pow(deltaTime,2), 0, 0, 0},
                {0, 0, 0, 0, 1, deltaTime, 0, 0, 0},
                {0, 0, 0, 0, 0, 1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 1, deltaTime, 0.5*Math.pow(deltaTime,2)},
                {0, 0, 0, 0, 0, 0, 0, 1, deltaTime},
                {0, 0, 0, 0, 0, 0, 0, 0, 1},
        });

        processNoise = new Array2DRowRealMatrix(new double[][]{
                {Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0, 0, 0, 0},
                {Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0,0, 0, 0},
                {0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0},
                {0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0},
                {0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2},
                {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime},
                {0, 0, 0, 0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1}
        }).scalarMultiply(processUncertainty);

        pm.updateMatrices(stateTransition, controlMatrix, processNoise,  initialState, resetStateCovarError);
        mm.updateMatrices(measurementMatrix, measurementUncertaintyMatrix);

        filter.updateMatrices();

    }

    public void setOdometry(boolean up) {
        odoRetractionController.set(up);
    }

    public double[] getDistances() {
        return distLocalizer.getDistances();
    }

    public Pose2d getDistEstimate() {
        return distEstimate;
    }

    public Pose2d getOdoEstimate() {
        return odoEstimate;
    }

    public Pose2d getWheelEstimate() {
        return wheelEstimate;
    }
}
