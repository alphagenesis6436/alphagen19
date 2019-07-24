package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Mechanism {

    protected Gamepad gamepad1, gamepad2;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected long startTime = 0; // in nanoseconds

    protected Mechanism() {}

    public void syncOpMode(Gamepad gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        gamepad1 = gamepad;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        startTime = System.nanoTime();
    }

    public void syncOpMode(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        syncOpMode(gamepad1, telemetry, hardwareMap);
        this.gamepad2 = gamepad2;
    }

    public abstract void init(String... deviceNames);

    public abstract void update();
}

interface Telemetrable {
    void telemetry();
}