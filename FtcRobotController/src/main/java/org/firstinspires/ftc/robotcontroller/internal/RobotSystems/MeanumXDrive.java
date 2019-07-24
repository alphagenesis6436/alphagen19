package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

public class MeanumXDrive extends DriveTrain {

    public void telemetry(){
        telemetry.addData("MECANUM X DRIVE", "TELEMETRY");
        telemetry.addData(">>>Front Left Pwr", motors.get(0).getPower());
        telemetry.addData(">>>Front Right Pwr", motors.get(1).getPower());
        telemetry.addData(">>>Back Left Pwr", motors.get(2).getPower());
        telemetry.addData(">>>Back Right Pwr", motors.get(3).getPower());
    }

    public void update(){
        motors.get(0).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax);
        motors.get(1).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax);
        motors.get(2).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax);
        motors.get(3).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax);
    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }
}
