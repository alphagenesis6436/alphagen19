package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ben on 9/29/2018.
 */

@TeleOp(name = "DriveTrainOp", group = "Default")
@Disabled
public class DriveTrainOp extends OpMode {
    //Declare any motors
    DcMotor FL; //left if forward
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    //Declare any variables & constants pertaining to drive train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    DriveMode driveMode = DriveMode.TANK;

    public DriveTrainOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        FL = hardwareMap.dcMotor.get("fl");
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL = hardwareMap.dcMotor.get("bl");
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("br");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry();
    }
    @Override public void loop() {
        //Update all the data based on driver input
        updateData();

        /* Clip Variables to make sure they don't exceed their
         * ranged values and Set them to the Motors/Servos */
        initialization();

        //Show the Real Values of the Data Using Telemetry
        telemetry();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateDriveTrain();
    }

    void initialization() {
        //Clip and Initialize Drive Train
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FL.setPower(currentLeftPwr);
        BL.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FR.setPower(currentRightPwr);
        BR.setPower(currentRightPwr);

    }
    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("Left Drive Pwr", FL.getPower());
        telemetry.addData("Right Drive Pwr", BR.getPower());
    }

    //Controlled by Driver 1
    //step 1: Push up/down the left/right stick to control the left/right drive motors
    void updateTankDrive() {
        currentLeftPwr = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        currentRightPwr = -gamepad1.right_stick_y * DRIVE_PWR_MAX;
    }

    //Controlled by Driver 1
    //step 1: Push Left Stick up or down to move robot forward or backward. Move Right Stick Left or Right
    void updateArcadeDrive() {
        currentLeftPwr = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        currentRightPwr = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
    }

    //Controlled by Driver 1
    //step 1: Press left/right bumper to select tank/arcade drive
    void updateDriveTrain() {
        if (gamepad1.left_bumper){
            driveMode = DriveMode.TANK;
        }
        else if (gamepad1.right_bumper){
            driveMode = DriveMode.ARCADE;
        }

        switch (driveMode) {
            case TANK: updateTankDrive();
                break;
            case ARCADE: updateArcadeDrive();
                break;
        }
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {

    }
    void runEncoders() {

    }
    void runWithoutEncoders() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}


