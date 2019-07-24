package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.Mechanism;

import java.util.ArrayList;

public class TieFighterOp extends TeleOpMode {

    TankDrive tankDrive = new TankDrive();
    FourBarLift fourBarLift = new FourBarLift();

    @Override
    void telemetry() {
        tankDrive.telemetry();
        fourBarLift.telemtry();
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
