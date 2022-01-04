package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class DistanceSensorArrayLocalizer implements Localizer {
    protected RingBuffer    lastEstimates = new RingBuffer(3),
                            lastTimes = new RingBuffer(3);

    protected MaxBoticsArray array;



    public DistanceSensorArrayLocalizer(MaxBoticsArray array) {
        this.array = array;

    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {

    }
}
