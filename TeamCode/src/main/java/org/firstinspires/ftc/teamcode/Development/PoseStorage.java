package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public enum Alliance {
        RED, BLUE, NEITHER
    }
    public enum StartingPosition {
        WAREHOUSE, DUCK, NOT_SET
    }

    public static Alliance alliance = Alliance.NEITHER;

    public static StartingPosition startingPosition = StartingPosition.NOT_SET;

    public static double armPos = 0;

    public static boolean isArrayInit = false;
}
