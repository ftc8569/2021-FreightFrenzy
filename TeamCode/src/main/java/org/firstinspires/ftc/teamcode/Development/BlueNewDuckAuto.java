package org.firstinspires.ftc.teamcode.Development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class BlueNewDuckAuto extends MainAutoV1 {

    @Override
    public void init() {
        initRed = false;
        initBlue = true;
        super.init();
    }

    @Override
    Position findPosition() {
        return new Position(PoseStorage.Alliance.BLUE, PoseStorage.StartingPosition.NEW_DUCK);
    }
}
