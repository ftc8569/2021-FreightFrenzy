package org.firstinspires.ftc.teamcode.Controllers;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    double dT = 0,
    v = 0,
    x = 0,
    y = 0,
    theta = 0,
    xV = 0,
    yV = 0,
    thetaV = 0,
    xA = 0,
    yA = 0,
    thetaA = 0,
    nextx = 0,
    nexty = 0,
    nexttheta = 0,
    nextxV = 0,
    nextyV = 0,
    nextthetaV = 0,
    nextxA = 0,
    nextyA = 0,
    nextthetaA = 0,
    px,
    py,
    ptheta,
    pxV,
    pyV,
    pthetaV,
    pxA,
    pyA,
    pthetaA,
    sigma2a;

    SimpleMatrix currentState = new SimpleMatrix(new double[9][1]),
                 predictedState = new SimpleMatrix(new double[9][1]);


    SimpleMatrix predicton = new SimpleMatrix(new double[][]{
            {1, dT, 0.5*Math.pow(dT,2), 0, 0, 0, 0, 0, 0},
            {0, 1, dT, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, dT, 0.5*Math.pow(dT,2), 0, 0, 0},
            {0, 0, 0, 0, 1, dT, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, dT, 0.5*Math.pow(dT,2)},
            {0, 0, 0, 0, 0, 0, 0, 1, dT},
            {0, 0, 0, 0, 0, 0, 0, 0, 1},
    });

    SimpleMatrix predictonbutweird = new SimpleMatrix(new double[][]{
            {1, 0, 0, 0, 0, 0, 0, 0, 0},
            {1, dT, 0, 0, 0, 0, 0, 0, 0},
            {1, dT, 0.5*Math.pow(dT,2), 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, dT, 0, 0, 0, 0},
            {0, 0, 0, 1, dT, 0.5*Math.pow(dT,2), 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, dT, 0},
            {0, 0, 0, 0, 0, 0, 1, dT, 0.5*Math.pow(dT,2)},
    });

    SimpleMatrix odometry = new SimpleMatrix(new double[][]{
            {1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 0, 0}
    });

    SimpleMatrix mecanum = new SimpleMatrix(new double[][]{
            {1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0}
    });

    SimpleMatrix vision = new SimpleMatrix(new double[][]{
            {1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0}
    });

    SimpleMatrix P = new SimpleMatrix(new double[][]{
            {px, px * pxV, px * pxA, 0, 0, 0, 0, 0, 0},
            {pxV * px, pxV, pxV * pxA, 0, 0, 0, 0, 0, 0},
            {pxA * px, pxA * pxV, pxA, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, py, py * pyV, py * pyA, 0, 0, 0},
            {0, 0, 0, pyV * py, pyV, pyV * pyA, 0, 0, 0},
            {0, 0, 0, pyA * py, pyA * pyV, pyA, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, ptheta, ptheta * pthetaV, ptheta * pthetaA},
            {0, 0, 0, 0, 0, 0, pthetaV * ptheta, pthetaV, pthetaV * pthetaA},
            {0, 0, 0, 0, 0, 0, pthetaA * ptheta, pthetaA * pthetaV, pthetaA},
    });

    SimpleMatrix Q = new SimpleMatrix(new double[][]{
            {Math.pow(dT,4)/4,Math.pow(dT,3)/2, Math.pow(dT,2)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {Math.pow(dT,3)/2, Math.pow(dT,2), dT, 0, 0, 0, 0, 0, 0},
            {Math.pow(dT,2)/2, dT, 1, 0, 0, 0,0, 0, 0},
            {0, 0, 0, Math.pow(dT,4)/4,Math.pow(dT,3)/2, Math.pow(dT,2)/2, 0, 0, 0},
            {0, 0, 0, Math.pow(dT,3)/2, Math.pow(dT,2), dT, 0, 0, 0},
            {0, 0, 0, Math.pow(dT,2)/2, dT, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, Math.pow(dT,4)/4,Math.pow(dT,3)/2, Math.pow(dT,2)/2},
            {0, 0, 0, 0, 0, 0, Math.pow(dT,3)/2, Math.pow(dT,2), dT},
            {0, 0, 0, 0, 0, 0, Math.pow(dT,2)/2, dT, 1},
    });

    SimpleMatrix pNext;

    SimpleMatrix meas;


    void covarExtrap() {
        SimpleMatrix q = Q.mult(new SimpleMatrix(new double[][]{{sigma2a}}));

        pNext = q.plus(predicton.mult(P.mult(predictonbutweird)));
    }


}
