package org.firstinspires.ftc.teamcode.Controllers;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NoDataException;
import org.apache.commons.math3.exception.NullArgumentException;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

//Almost exactly DefaultProcessModel but with one method added to edit the matrices
public class AdjustableProcessModel implements ProcessModel {

    /**
     * The state transition matrix, used to advance the internal state estimation each time-step.
     */
    private RealMatrix stateTransitionMatrix;

    /**
     * The control matrix, used to integrate a control input into the state estimation.
     */
    private RealMatrix controlMatrix;

    /** The process noise covariance matrix. */
    private RealMatrix processNoiseCovMatrix;

    /** The initial state estimation of the observed process. */
    private RealVector initialStateEstimateVector;

    /** The initial error covariance matrix of the observed process. */
    private RealMatrix initialErrorCovMatrix;

    /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @param initialStateEstimate
     *            the initial state estimate vector
     * @param initialErrorCovariance
     *            the initial error covariance matrix
     * @throws NullArgumentException
     *             if any of the input arrays is {@code null}
     * @throws NoDataException
     *             if any row / column dimension of the input matrices is zero
     * @throws DimensionMismatchException
     *             if any of the input matrices is non-rectangular
     */
    public AdjustableProcessModel(final double[][] stateTransition,
                                  final double[][] control,
                                  final double[][] processNoise,
                                  final double[] initialStateEstimate,
                                  final double[][] initialErrorCovariance)
            throws NullArgumentException, NoDataException, DimensionMismatchException {

        this(new Array2DRowRealMatrix(stateTransition),
                new Array2DRowRealMatrix(control),
                new Array2DRowRealMatrix(processNoise),
                new ArrayRealVector(initialStateEstimate),
                new Array2DRowRealMatrix(initialErrorCovariance));
    }

    /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     * <p>
     * The initial state estimate and error covariance are omitted and will be initialized by the
     * {@link KalmanFilter} to default values.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @throws NullArgumentException
     *             if any of the input arrays is {@code null}
     * @throws NoDataException
     *             if any row / column dimension of the input matrices is zero
     * @throws DimensionMismatchException
     *             if any of the input matrices is non-rectangular
     */
    public AdjustableProcessModel(final double[][] stateTransition,
                                  final double[][] control,
                                  final double[][] processNoise)
            throws NullArgumentException, NoDataException, DimensionMismatchException {

        this(new Array2DRowRealMatrix(stateTransition),
                new Array2DRowRealMatrix(control),
                new Array2DRowRealMatrix(processNoise), null, null);
    }

    /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @param initialStateEstimate
     *            the initial state estimate vector
     * @param initialErrorCovariance
     *            the initial error covariance matrix
     */
    public AdjustableProcessModel(final RealMatrix stateTransition,
                               final RealMatrix control,
                               final RealMatrix processNoise,
                               final RealVector initialStateEstimate,
                               final RealMatrix initialErrorCovariance) {
        this.stateTransitionMatrix = stateTransition;
        this.controlMatrix = control;
        this.processNoiseCovMatrix = processNoise;
        this.initialStateEstimateVector = initialStateEstimate;
        this.initialErrorCovMatrix = initialErrorCovariance;
    }

    /** {@inheritDoc} */
    public RealMatrix getStateTransitionMatrix() {
        return stateTransitionMatrix;
    }

    /** {@inheritDoc} */
    public RealMatrix getControlMatrix() {
        return controlMatrix;
    }

    /** {@inheritDoc} */
    public RealMatrix getProcessNoise() {
        return processNoiseCovMatrix;
    }

    /** {@inheritDoc} */
    public RealVector getInitialStateEstimate() {
        return initialStateEstimateVector;
    }

    /** {@inheritDoc} */
    public RealMatrix getInitialErrorCovariance() {
        return initialErrorCovMatrix;
    }

    public void updateMatrices(final RealMatrix stateTransition,
                          final RealMatrix control,
                          final RealMatrix processNoise,
                          final RealVector initialStateEstimate,
                          final RealMatrix initialErrorCovariance) {
        this.stateTransitionMatrix = stateTransition;
        this.controlMatrix = control;
        this.processNoiseCovMatrix = processNoise;
        this.initialStateEstimateVector = initialStateEstimate;
        this.initialErrorCovMatrix = initialErrorCovariance;
    }
}