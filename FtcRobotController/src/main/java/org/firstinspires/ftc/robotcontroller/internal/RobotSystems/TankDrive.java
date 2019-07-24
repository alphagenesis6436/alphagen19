package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

import org.firstinspires.ftc.robotcontroller.internal.TeleOpMode;

public class TankDrive extends DriveTrain{

    public void telemetry(){
        telemetry.addData("TANK DRIVE", "TELEMETRY");
        telemetry.addData(">>>Left Pwr", motors.get(0).getPower());
        telemetry.addData(">>>Right Pwr", motors.get(1).getPower());
    }

    public void update(){
        for(int i = 0; i < motors.size(); i++)
            motors.get(i).setPower(clipPower((i%2==0)?-gamepad1.left_stick_y * drivePwrMax:-gamepad1.right_stick_y * drivePwrMax));
    }

    @Override
    public void move(double... velocities) {

    }

    public void moveForward(double speed){
    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }
}
