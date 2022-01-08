package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * A localizer that uses an array of distance sensors to get a very accurate position reading in
 * specific sitautions. Note that the refresh rate of the sensors does not make this viable for
 * velocity tracking (maybe if you really filter it).
 */
public class DistanceSensorArrayLocalizer implements Localizer {

    protected Pose2d lastEstimate = new Pose2d();

    protected MaxBoticsArray array;

    protected Pose2d bestEstimate = new Pose2d();

    protected final int[] headings = {-180, -90, 0, 90, 180};

    public DistanceSensorArrayLocalizer(MaxBoticsArray array) {
        this.array = array;
    }

    public enum Corner  {
            TopLeft, TopRight,
            BackLeft, BackRight
    }

    public static double frontOffset = 9, backOffset = 9, leftOffset = 7, rightOffset = 7;

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return lastEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        bestEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        double[] distances = array.getDistances(DistanceUnit.INCH);

        if(Math.abs(bestEstimate.getX()) > 24 && Math.abs(bestEstimate.getY()) > 24 && (Math.abs(Math.toDegrees(bestEstimate.getHeading())) % 90 < 45) ? Math.abs(Math.toDegrees(bestEstimate.getHeading())) % 90 < 3 :  90 - Math.abs(Math.toDegrees(bestEstimate.getHeading()) % 90) < 3) {
            double val = bestEstimate.getHeading();
            double distance = Math.abs(headings[0] - val);
            int id = 0;
            for(int c = 1; c < headings.length; c++){
                double cdistance = Math.abs(headings[c] - val);
                if(cdistance < distance){
                    id = c;
                    distance = cdistance;
                }
            }
            int orientation = headings[id];

            int xSign = (int) Math.signum(bestEstimate.getX());
            int ySign = (int) Math.signum(bestEstimate.getY());

            Corner currCorner;
            if(xSign > 0) {
                if(ySign > 0) {
                    currCorner = Corner.TopLeft;
                } else currCorner = Corner.TopRight;
            } else if(ySign > 0) {
                currCorner = Corner.BackLeft;
            } else currCorner = Corner.BackRight;

            switch (currCorner) {
                case TopLeft:
                    switch (orientation) {
                        case 0:
                            if(distances[0] < 48 && distances[2] < 48) lastEstimate = new Pose2d(72 - distances[0] - frontOffset, 72 - distances[2] - leftOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 90:
                            if(distances[3] < 48 && distances[0] < 48) lastEstimate = new Pose2d(72 - distances[3] - rightOffset, 72 - distances[0] - frontOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 180:
                        case -180:
                            if(distances[1] < 48 && distances[3] < 48) lastEstimate = new Pose2d(72 - distances[1] - backOffset, 72 - distances[3] - rightOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case -90:
                            if(distances[2] < 48 && distances[1] < 48) lastEstimate = new Pose2d(72 - distances[2] - leftOffset, 72 - distances[1] - backOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;
                    }
                    break;

                case TopRight:
                    switch (orientation) {
                        case 0:
                            if(distances[0] < 48 && distances[3] < 48) lastEstimate = new Pose2d(72 - distances[0] - frontOffset, -72 + distances[3] + rightOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 90:
                            if(distances[3] < 48 && distances[1] < 48) lastEstimate = new Pose2d(72 - distances[3] - rightOffset, -72 + distances[1] + backOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 180:
                        case -180:
                            if(distances[1] < 48 && distances[2] < 48) lastEstimate = new Pose2d(72 - distances[1] - backOffset, -72 + distances[2] + leftOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case -90:
                            if(distances[2] < 48 && distances[0] < 48) lastEstimate = new Pose2d(72 - distances[2] - leftOffset, -72 + distances[0] + frontOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;
                    }
                    break;

                case BackLeft:
                    switch (orientation) {
                        case 0:
                            if(distances[1] < 48 && distances[2] < 48) lastEstimate = new Pose2d(-72 + distances[1] + backOffset, 72 - distances[2] - leftOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 90:
                            if(distances[2] < 48 && distances[0] < 48) lastEstimate = new Pose2d(-72 + distances[2] + leftOffset, 72 - distances[0] - frontOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 180:
                        case -180:
                            if(distances[0] < 48 && distances[3] < 48) lastEstimate = new Pose2d(-72 + distances[0] + frontOffset, 72 - distances[3] - rightOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case -90:
                            if(distances[3] < 48 && distances[1] < 48) lastEstimate = new Pose2d(72 + distances[3] + rightOffset, 72 - distances[1] - backOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;
                    }
                    break;

                case BackRight:
                    switch (orientation) {
                        case 0:
                            if(distances[1] < 48 && distances[3] < 48) lastEstimate = new Pose2d(-72 + distances[1] + backOffset, -72 + distances[3] + rightOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 90:
                            if(distances[2] < 48 && distances[1] < 48) lastEstimate = new Pose2d(-72 + distances[2] + leftOffset, -72 + distances[1] + backOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case 180:
                        case -180:
                            if(distances[0] < 48 && distances[2] < 48) lastEstimate = new Pose2d(-72 + distances[0] + frontOffset, -72 + distances[2] + leftOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;

                        case -90:
                            if(distances[3] < 48 && distances[0] < 48) lastEstimate = new Pose2d(72 + distances[3] + rightOffset, -72 + distances[1] + frontOffset, bestEstimate.getHeading());
                            else lastEstimate = new Pose2d();
                            break;
                    }
                    break;
            }

        } else lastEstimate = new Pose2d();
    }
}
