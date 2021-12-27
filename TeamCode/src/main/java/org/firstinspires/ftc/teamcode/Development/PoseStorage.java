package org.firstinspires.ftc.teamcode.Development;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public enum Alliance {
        RED, BLUE, NEITHER
    }
    public static Alliance alliance = Alliance.NEITHER;
}
