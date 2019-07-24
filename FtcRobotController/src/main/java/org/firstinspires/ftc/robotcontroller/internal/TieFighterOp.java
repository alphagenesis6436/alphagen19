package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.Mechanism;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.TankDrive;

import java.util.ArrayList;

public class TieFighterOp extends TeleOpMode {

    TankDrive tankDrive = new TankDrive();

    @Override
    void telemetry() {
        tankDrive.telemetry();
    }

    @Override
    void updateData() {
        tankDrive.update();
    }

    @Override
    public void init() {
        tankDrive.init("lm", "rm");
    }
}
