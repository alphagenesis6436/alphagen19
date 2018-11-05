package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sabrina on 10/13/2018.
 */

@TeleOp(name = "LatchTileOp", group = "Default")
//@Disabled
public class LatchTileOp extends OpMode {
    //Declare any motors
    DcMotor FL; //left if forward
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor LA;
    //Declare any variables & constants pertaining to drive train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    final double LATCH_PWR = 0.80;
    double currentLatchPwr = 0.0;
    DriveMode driveMode = DriveMode.TANKDRIVE;

    public LatchTileOp() {}

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
        LA = hardwareMap.dcMotor.get("la");
        LA.setDirection(DcMotorSimple.Direction.FORWARD);

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
        updateLatch();
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
            driveMode = DriveMode.TANKDRIVE;
        }
        else if (gamepad1.right_bumper){
            driveMode = DriveMode.ARCADEDRIVE;
        }

        switch (driveMode) {
            case TANKDRIVE: updateTankDrive();
                break;
            case ARCADEDRIVE: updateArcadeDrive();
                break;
        }
    }
    void updateLatch() {
        currentLatchPwr = 0.0;
        if(gamepad1.dpad_up) {
            currentLatchPwr = LATCH_PWR;
        }
        else if (gamepad1.dpad_down) {
            currentLatchPwr = -LATCH_PWR;
        }
    }


    void initialization() {
        //Clip and Initialize Drive Train
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FL.setPower(currentLeftPwr);
        BL.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FR.setPower(currentRightPwr);
        BR.setPower(currentRightPwr);
        currentLatchPwr = Range.clip(currentLatchPwr, -LATCH_PWR, LATCH_PWR);
        LA.setPower(currentLatchPwr);

    }
    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("Left Drive Pwr", FL.getPower());
        telemetry.addData("Right Drive Pwr", BR.getPower());
        telemetry.addData("Latch Pwr", LA.getPower());

    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */




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
