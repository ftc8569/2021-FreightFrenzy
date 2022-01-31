package org.firstinspires.ftc.teamcode.Controllers;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NoDataException;
import org.apache.commons.math3.exception.NullArgumentException;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class AdjustableMeasurementModel implements MeasurementModel {


    /**
     * The measurement matrix, used to associate the measurement vector to the
     * internal state estimation vector.
     */
    private RealMatrix measurementMatrix;

    /**
     * The measurement noise covariance matrix.
     */
    private RealMatrix measurementNoise;

    /**
     * Create a new {@link MeasurementModel}, taking double arrays as input parameters for the
     * respective measurement matrix and noise.
     *
     * @param measMatrix
     *            the measurement matrix
     * @param measNoise
     *            the measurement noise matrix
     * @throws NullArgumentException
     *             if any of the input matrices is {@code null}
     * @throws NoDataException
     *             if any row / column dimension of the input matrices is zero
     * @throws DimensionMismatchException
     *             if any of the input matrices is non-rectangular
     */
    public AdjustableMeasurementModel(final double[][] measMatrix, final double[][] measNoise)
            throws NullArgumentException, NoDataException, DimensionMismatchException {
        this(new Array2DRowRealMatrix(measMatrix), new Array2DRowRealMatrix(measNoise));
    }

    /**
     * Create a new {@link MeasurementModel}, taking {@link RealMatrix} objects
     * as input parameters for the respective measurement matrix and noise.
     *
     * @param measMatrix the measurement matrix
     * @param measNoise the measurement noise matrix
     */
    public AdjustableMeasurementModel(final RealMatrix measMatrix, final RealMatrix measNoise) {
        this.measurementMatrix = measMatrix;
        this.measurementNoise = measNoise;
    }

    /** {@inheritDoc} */
    public RealMatrix getMeasurementMatrix() {
        return measurementMatrix;
    }

    /** {@inheritDoc} */
    public RealMatrix getMeasurementNoise() {
        return measurementNoise;
    }

    public void updateMatrices(final RealMatrix measMatrix, final RealMatrix measNoise) {
        this.measurementMatrix = measMatrix;
        this.measurementNoise = measNoise;
    }
}
