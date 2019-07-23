package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public abstract class DriveTrain extends Mechanism implements Telemetrable {

    /** All constants necessary for FTC drive trains */

    public static final int COUNTS_PER_REVOLUTION_40 = 1120; //AndyMark 40:1 Motors

    /** All fields necessary for FTC drive trains */

    // Drive Motors: 0 -> frontLeft, 1 -> frontRight, 2 -> backLeft, 3 -> backRight
    protected ArrayList<DcMotor> motors = new ArrayList<>();
    protected int numOfMotors; //Only 2 and 4 are accepted values
    protected double drivePwrMax = 0.80; //80% by default
    protected double turnPwrMax = 0.60; //60% by default, only for slide/mecanum/holonomic
    protected double gearRatio = 1; //Driven / Driver, 1 by default

    /** Protected no-arg constructor */

    protected DriveTrain() { }

    /** Accessor & Mutator Methods for class fields */

    protected void setNumOfMotors(int num) {
        numOfMotors = num;
    }
    public void setDrivePwrMax(double maxPwr) {
        drivePwrMax = Range.clip(maxPwr, -1, 1);
    }
    public void setTurnPwrMax(double maxPwr) {
        turnPwrMax = Range.clip(maxPwr, -1, 1);
    }
    public void setGearRatio(double ratio) {
        gearRatio = ratio;
    }

    /** Set Multiple fields at once */

    public void setConstants(double drivePwrMax, double turnPwrMax, double gearRatio) {
        setDrivePwrMax(drivePwrMax);
        setTurnPwrMax(turnPwrMax);
        setGearRatio(gearRatio);
    }
    public void setConstants(double drivePwrMax, double gearRatio) {
        setDrivePwrMax(drivePwrMax);
        setGearRatio(gearRatio);
    }

    /** init() method for all drive trains. user must input several strings of
     * motor names in order from front to back then left to right. For example:
     * Drive Motors: 0 -> frontLeft, 1 -> frontRight, 2 -> backLeft, 3 -> backRight */

    @Override
    public void init(String... motorNames) {
        setNumOfMotors(motorNames.length);
        for (int i = 0; i < numOfMotors; i++) {
            motors.add(hardwareMap.dcMotor.get(motorNames[i]));
            // set the direction of the motors
            if (i % 2 == 0) motors.get(i).setDirection(DcMotorSimple.Direction.REVERSE);
            else motors.get(i).setDirection(DcMotorSimple.Direction.FORWARD);
        }
        telemetry.addData(">", "DriveTrain.init() Successful");
    }

    protected double clipPower(double power) {
        return Range.clip(power, -drivePwrMax, drivePwrMax);
    }

    private void updateMecanumXDrive() {
        frPower = (-gamepad.left_stick_y - gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        flPower = (-gamepad.left_stick_y + gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        brPower = (-gamepad.left_stick_y + gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        blPower = (-gamepad.left_stick_y - gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
    }
    private void updateMecanumODrive() {
        frPower = (-gamepad.left_stick_y + gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        flPower = (-gamepad.left_stick_y - gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        brPower = (-gamepad.left_stick_y - gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        blPower = (-gamepad.left_stick_y + gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
    }
    private void updateHolonomicDrive() {
        if (!(gamepad.right_stick_x < 0.03 && gamepad.right_stick_x > -0.03)) { //Move Clockwise/Anticlockwise
            flPower = gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            frPower = -gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            blPower = gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            brPower = -gamepad.right_stick_x * drivePwrMax * turnPwrMax;
        }
        else if (!(gamepad.left_stick_y < 0.10 && gamepad.left_stick_y > -0.10) &&
                !(gamepad.left_stick_x < 0.10 && gamepad.left_stick_x > -0.10)) { //Move at diagonal
            if ((gamepad.left_stick_x > 0 && gamepad.left_stick_y < 0) ||
                    (gamepad.left_stick_x < 0 && gamepad.left_stick_y > 0)) { //ForwardRight/BackwardLeft
                flPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                frPower = (-gamepad.left_stick_y - gamepad.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                blPower = (-gamepad.left_stick_y - gamepad.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                brPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                if (gamepad.left_stick_x < 0) {
                    flPower *= -1;
                    brPower *= -1;
                }
            }
            else { //ForwardLeft/BackwardRight
                flPower = (-gamepad.left_stick_y + gamepad.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                frPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                blPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                brPower = (-gamepad.left_stick_y + gamepad.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                if (gamepad.left_stick_x > 0) {
                    frPower *= -1;
                    blPower *= -1;
                }
            }
        }
        else if (!(gamepad.left_stick_y < 0.10 && gamepad.left_stick_y > -0.10)) { //Move forward/backward
            flPower = -gamepad.left_stick_y * drivePwrMax;
            frPower = -gamepad.left_stick_y * drivePwrMax;
            blPower = -gamepad.left_stick_y * drivePwrMax;
            brPower = -gamepad.left_stick_y * drivePwrMax;
        }
        else if (!(gamepad.left_stick_x < 0.10 && gamepad.left_stick_x > -0.10)) { //Move Right/left
            flPower = gamepad.left_stick_x * drivePwrMax;
            frPower = -gamepad.left_stick_x * drivePwrMax;
            blPower = -gamepad.left_stick_x * drivePwrMax;
            brPower = gamepad.left_stick_x * drivePwrMax;
        }
        else if (gamepad.right_stick_x == 0 && gamepad.left_stick_y == 0 && gamepad.left_stick_x == 0) { //Stop Motion
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }

    }


    //Class variables for Autonomous Use
    private BNO055IMU imu; //For detecting rotation
    private Orientation angles;
    private PID anglePID = new PID();
    private boolean disableEncoderCalibration = false;
    public boolean encoderTargetReached = false;
    public boolean angleTargetReached = false;
    public EncoderMode encoderMode = EncoderMode.CONSTANT_SPEED;
    private long startTime = 0; // in nanoseconds

    //Methods for Autonomous
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }
    public void resetStartTime() {
        startTime = System.nanoTime();
    }
    protected void initializeIMU() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }
    public void startIMU() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    private void updateAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public float getHeading() {
        updateAngles();
        telemetry.addData("Heading", -angles.firstAngle);
        return -angles.firstAngle;
    }

    private int getEncoderCount() { return motors.get(0).getCurrentPosition(); }

    public void resetEncoders() {
        for (DcMotor motor: motors)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void runConstantSpeed() {
        for (DcMotor motor: motors)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    protected void runConstantPower() {
        for (DcMotor motor: motors)
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDriveMotors() {
        for (DcMotor motor: motors)
            motor.setPower(0);
    }

    public abstract void move(double... velocities);

    public abstract void moveForward(double velocity);
    public void moveForward(double velocity, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION_40 * gearRatio;
        double kp = 2 * (Math.abs(velocity) - 0.10) / COUNTS_PER_REVOLUTION_40;
        double error = target - getEncoderCount();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION_40 / 2) {
                velocity = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForward(velocity);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION_40 / gearRatio));
        }
    }

    public abstract void turnClockwise(double angularVelocity);
    public void turnClockwise(int targetAngle) {
        updateAngles();
        telemetry.addData("Heading", String.format("%.0f", getHeading()));
        double k = 0.005; //experimentally found
        double e = targetAngle + angles.firstAngle; //clockwise is negative for firstAngle
        double angularVelocity = (0.05 * e / Math.abs(e)) + k * e;
        angularVelocity = Range.clip(angularVelocity, -1.0, 1.0);
        if (Math.abs(e) >= 2)
            turnClockwise(angularVelocity);
        else {
            stopDriveMotors();
            angleTargetReached = true;
        }
    }

    public void turnClockwisePID(int targetAngle) {
        updateAngles();
        telemetry.addData("Heading", String.format("%.0f", getHeading()));
        anglePID.setTargetValue(targetAngle);
        anglePID.update(-angles.firstAngle, getRuntime());
        double angularVelocity = anglePID.adjustmentValue();
        angularVelocity = Range.clip(angularVelocity, -1, 1); //ensure power doesn't exceed max speed
        if (Math.abs(anglePID.getError()) >= 5) //5 degree angle slack / uncertainty
            turnClockwise(angularVelocity);
        else {
            stopDriveMotors();
            anglePID.reset();
            angleTargetReached = true;
        }

    }
}

//This enum allows us to switch between Encoder Modes for Autonomous
enum EncoderMode {
    CONSTANT_SPEED, CONSTANT_POWER;
}
