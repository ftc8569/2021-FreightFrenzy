package org.firstinspires.ftc.teamcode.Development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "TeleOPV1")
public class RedWarehouseAuto extends MainAutoV1{

    @Override
    Position findPosition() {
        return new Position(PoseStorage.Alliance.RED, PoseStorage.StartingPosition.WAREHOUSE);
    }
}
