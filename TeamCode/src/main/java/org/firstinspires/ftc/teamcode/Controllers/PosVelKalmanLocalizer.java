package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.NonSymmetricMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.HashMap;

@Config
public class PosVelKalmanLocalizer implements Localizer {
    public static double processUncertainty = 20;

   //we will currently assume our velocity uncertainty is the same as our position.
            // This is not true but I do not think we will pass it velocity - the localizer is weird.
    public static double measUncOdo = 1, measUncDist = .25, measUncWheels = 5;

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

    private int errors = 0;

    public double heading = 0;

    private double offset = 0;

    RealMatrix stateTransition;
    RealMatrix controlMatrix = null;

    //TODO: FIGURE THIS OUT A LITTLE BETTER
    //guesstimate of the std dev of our acceleration model in in/s^2

    RealMatrix processNoise;

    RealVector initialState = new ArrayRealVector(new double[] {
            0, 0, 0, 0
    });

    //TODO: this probably shouldn't be 0 but we do know pretty exactly our start pos.
    // We know we are not moving though

    //Although, we know we are not moving at first, but what if we reset our position later?
    // Our position will be accurate but we don't know if our velocity will be accurate so we
    // might add some error there for our later positons.
    double initialStateError = 0.5;

    double resetStateError = 5;

    RealMatrix initialStateCovarError;

    RealMatrix resetStateCovarError;

    AdjustableProcessModel pm;

    RealMatrix measurementMatrix = new Array2DRowRealMatrix(new double[][]{
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1},
    });



    double measurementUncertainty = measUncOdo;

    //troubleshooting, ignore weird values
    //TODO: do some bench top measurements on our uncertainty here. A bit hard to do but we can get the idea.
    RealMatrix measurementUncertaintyMatrix;

    AdjustableMeasurementModel mm;

    Pose2d odoEstimate = new Pose2d(0, 0, 0),
            wheelEstimate = new Pose2d(0, 0, 0), distEstimate = new Pose2d(0, 0, 0),
            odoVeloEstimate = new Pose2d(0, 0, 0),wheelVeloEstimate = new Pose2d(0, 0, 0);

    double[] stateEstimate = new double[]{0, 0, 0, 0};

    //Vectors to give to correct()

    //TODO: maybe make a variable to control whether or not to give velo information
    //to stop giving velo information, swap the top line for the bottom commented one for right now
    RealVector odoMeasureVector;

    //to stop giving pose information, swap the last 3 values for 0s
    RealVector wheelMeasureVector;

    RealVector distMeasureVector;


    public PosVelKalmanLocalizer(@NotNull SampleMecanumDrive drive, @NotNull TwoWheelTrackingLocalizer odoLocalizer,
                                 @NotNull MecanumDrive.MecanumLocalizer wheelLocalizer,
                                 @NotNull DistanceSensorArrayLocalizer distLocalizer, @NotNull OdoRetractionController odoRetractionController) {
        this.drive = drive;
        this.odoLocalizer = odoLocalizer;
        this.wheelLocalizer = wheelLocalizer;
        this.distLocalizer = distLocalizer;
        this.odoRetractionController = odoRetractionController;

        timer = new ElapsedTime();

        heading = drive.getRawExternalHeading();

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
                pose2d.getX(), 0, 0, pose2d.getY(), 0, 0
        });
        offset = Angle.normDelta(heading - pose2d.getHeading());
        heading = Angle.normDelta(drive.getRawExternalHeading() - offset);
        updateFilter();
        wheelLocalizer.setPoseEstimate(pose2d);
        odoLocalizer.setPoseEstimate(pose2d);
        distLocalizer.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(stateEstimate[1], stateEstimate[4], drive.getExternalHeadingVelocity() != null ? drive.getExternalHeadingVelocity() : 0);
    }

    @Override
    public void update() {
        if(!init) {
            deltaTime = 0;
            init = true;
        } else deltaTime = timer.seconds();

        heading = Angle.normDelta(drive.getRawExternalHeading() - offset);

        odoLocalizer.update();
        wheelLocalizer.update();


        measurementUncertainty = measUncWheels;

        wheelEstimate = wheelLocalizer.getPoseEstimate();
        wheelVeloEstimate = wheelLocalizer.getPoseVelocity() != null ? wheelLocalizer.getPoseVelocity() : new Pose2d(0, 0, 0);

        filter.predict();

        timer.reset();

        updateVectors();

        try{
            filter.correct(wheelMeasureVector);
        } catch (NonSymmetricMatrixException e) {
            errors++;
            e.printStackTrace();
        }

        measurementUncertainty = measUncOdo;

        odoEstimate = odoRetractionController.isUp() ? new Pose2d() : odoLocalizer.getPoseEstimate();
        odoVeloEstimate = odoLocalizer.getPoseVelocity() != null ? odoLocalizer.getPoseVelocity() : new Pose2d(0, 0, 0);


        if(!odoEstimate.equals(new Pose2d())) {
            deltaTime = timer.seconds();
            filter.predict();
            updateVectors();
            timer.reset();
            try {
            filter.correct(odoMeasureVector);
            } catch (NonSymmetricMatrixException e) {
                errors++;
                e.printStackTrace();
            }
        }

        measurementUncertainty = measUncDist;


        distLocalizer.setPoseEstimate(bestEstimate);
//        distLocalizer.setPoseVelocity(Objects.requireNonNull(getPoseVelocity()));
        distLocalizer.update();
        distEstimate = distLocalizer.getPoseEstimate();

        if(!distEstimate.equals(new Pose2d())) {
            deltaTime = timer.seconds();
            filter.predict();
            updateVectors();
            try {
            filter.correct(distMeasureVector);
            } catch (NonSymmetricMatrixException e) {
                errors++;
                e.printStackTrace();
            }
        }

        odoEstimate = odoLocalizer.getPoseEstimate();

        deltaTime = timer.seconds();
        filter.predict();
        updateVectors();


        HashMap<String, Object> map = new HashMap<>();
//        map.put("Heading Estimate", String.valueOf(headingMeasureVector.getEntry(5)));
        map.put("state Estimate", Arrays.toString(stateEstimate));
        timer.reset();
        stateEstimate = filter.getStateEstimation();
        bestEstimate = new Pose2d(stateEstimate[0], stateEstimate[3], heading);
    }

    private void updateFilter() {
        pm = new AdjustableProcessModel(stateTransition, controlMatrix, processNoise,  initialState, resetStateCovarError);
        mm = new AdjustableMeasurementModel(measurementMatrix, measurementUncertaintyMatrix);
        filter = new AdjustableKalmanFilter(pm, mm);
    }

    //TODO: do this better. DoubleSuppliers?
    private void updateVectors() {


        odoMeasureVector = new ArrayRealVector(new double[]{
//                odoEstimate.getX(), odoVeloEstimate.getX(), odoEstimate.getY(), odoVeloEstimate.getY(), odoEstimate.getHeading(), odoVeloEstimate.getHeading()
          odoEstimate.getX(), odoVeloEstimate.getX(), odoEstimate.getY(), odoVeloEstimate.getY()
        });

        wheelMeasureVector = new ArrayRealVector(new double[]{
//                wheelEstimate.getX(), wheelVeloEstimate.getX(), wheelEstimate.getY(), wheelVeloEstimate.getY(), wheelEstimate.getHeading(), wheelVeloEstimate.getHeading()
          wheelEstimate.getX(), wheelVeloEstimate.getX(), wheelEstimate.getY(), wheelVeloEstimate.getY()
        });
        distMeasureVector = new ArrayRealVector(new Double[]{
                distEstimate.getX(), 0.0, distEstimate.getY(), 0.0
        });

        processNoise = new Array2DRowRealMatrix(new double[][]{
                {Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0},
                {Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0},
                {Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0},
                {0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2},
                {0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime},
                {0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1}
        }).scalarMultiply(processUncertainty);

        resetStateCovarError = new Array2DRowRealMatrix(new double[][] {
                {initialStateError, 0, 0, 0, 0, 0},
                {0, resetStateError, 0, 0, 0, 0},
                {0, 0, resetStateError, 0, 0, 0},
                {0, 0, 0, initialStateError, 0, 0},
                {0, 0, 0, 0, resetStateError, 0},
                {0, 0, 0, 0, 0, resetStateError}
        });




        //we have to recalculate our matrices that are dependant on deltaT every loop and put them into our filter.
        stateTransition = new Array2DRowRealMatrix(new double[][] {
                {1, deltaTime, 0.5*Math.pow(deltaTime,2), 0, 0, 0 },
                {0, 1, deltaTime, 0, 0, 0},
                {0, 0, 1, 0, 0, 0},
                {0, 0, 0, 1, deltaTime, 0.5*Math.pow(deltaTime,2)},
                {0, 0, 0, 0, 1, deltaTime},
                {0, 0, 0, 0, 0, 1},
        });

        processNoise = new Array2DRowRealMatrix(new double[][]{
                {Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2, 0, 0, 0},
                {Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime, 0, 0, 0},
                {Math.pow(deltaTime,2)/2, deltaTime, 1, 0, 0, 0},
                {0, 0, 0, Math.pow(deltaTime,4)/4,Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2)/2},
                {0, 0, 0, Math.pow(deltaTime,3)/2, Math.pow(deltaTime,2), deltaTime},
                {0, 0, 0, Math.pow(deltaTime,2)/2, deltaTime, 1}
        }).scalarMultiply(processUncertainty);

        measurementUncertaintyMatrix = new Array2DRowRealMatrix(new double[][]{
                {Math.pow(measurementUncertainty, 2), 0, 0, 0},
                {0, Math.pow(measurementUncertainty, 2), 0, 0},
                {0, 0, Math.pow(measurementUncertainty, 2), 0},
                {0, 0, 0, Math.pow(measurementUncertainty, 2)},

        });

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

    public int getErrors() {
        return errors;
    }
}
