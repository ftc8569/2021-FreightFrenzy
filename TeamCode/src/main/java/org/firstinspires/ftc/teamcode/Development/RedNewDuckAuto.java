package org.firstinspires.ftc.teamcode.Development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class RedNewDuckAuto extends MainAutoV1 {
    @Override
    public void init() {
        initRed = true;
        initBlue = false;
        super.init();
    }
    @Override
    Position findPosition() {
        return new Position(PoseStorage.Alliance.RED, PoseStorage.StartingPosition.NEW_DUCK);
    }
}
