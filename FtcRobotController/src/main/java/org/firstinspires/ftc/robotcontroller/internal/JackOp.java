package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Jack 8/29/2018
 * blue rangerbot
 */

@TeleOp(name = "OpMode Template", group = "Default")
@Disabled
public class JackOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor leftDrive;
    DcMotor rightDrive;


    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    final double DRIVE_PWR_MAX = 0.8;

    public JackOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftDrive = hardwareMap.dcMotor.get("ld");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.dcMotor.get("rd");
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize servos

        //Initialize sensors

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
        currentLeftPwr = -gamepad1.left_stick_y*DRIVE_PWR_MAX;
        leftDrive.setPower(currentLeftPwr);
        currentRightPwr = -gamepad1.right_stick_y*DRIVE_PWR_MAX;
        rightDrive.setPower(currentRightPwr);

    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        leftDrive.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        rightDrive.setPower(currentRightPwr);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("LDP", leftDrive.getPower());
        telemetry.addData("RDP", rightDrive.getPower());

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
    void runConstantSpeed() {

    }
    void runConstantPower() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

