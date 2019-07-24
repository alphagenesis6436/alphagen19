package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class ArcadeDrive extends DriveTrain{
    @Override
    public void update() {
                double flPower, frPower;
                flPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * drivePwrMax;
                frPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * drivePwrMax;
                for (int i = 0; i < numOfMotors; i++) {
                    if (i % 2 == 0) {
                        motors.get(i).setPower(clipPower(flPower));
                    } else {
                        motors.get(i).setPower(clipPower(frPower));
                    }
                }
    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void telemetry() {
        telemetry.addData("DriveTrain","TankDrive");
        telemetry.addData("LeftPwr", motors.get(0).getPower());
        telemetry.addData("RightPwr", motors.get(1).getPower());
    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }
}
