package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.TankDrive;

public class KietOp extends OpMode {

    TankDrive tankDrive = new TankDrive();

    @Override
    public void init() {
        tankDrive.init("frontLeftMotor", "frontRightMotor");
    }

    @Override
    public void loop() {
        tankDrive.update();
        tankDrive.telemetry();
    }
}
