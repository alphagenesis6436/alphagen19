package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Update by Jack on 10/12/2018.
 */

@TeleOp(name = "SwingOp", group = "Default")
//@Disabled
public class SwingOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor swingMotor;


    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    double swingPow = 0;
    final double MAX_POW = 0.8;
    final int COUNTS_PER_REV = 1120;
    boolean p = false; //p for motor position(false = 0 rev, true = 2 rev)
    int swingState = 0;
    double swingPos = 0;


    public SwingOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        swingMotor = hardwareMap.dcMotor.get("sm");
        swingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        updateSwing();
    }

    void updateSwing() {
        switch (swingState) {
            case 0:
                swingPow = 0;
                if (gamepad1.x) {
                    swingState++;
                    setTime = this.time;
                }
                break;
            case 1:
                swingPow = MAX_POW;
                swingPos = swingMotor.getCurrentPosition();
                if (swingPos >= COUNTS_PER_REV * 0.8) {
                    swingState++;
                    setTime = this.time;
                }
                break;
            case 2:
                swingPow = 0;
                if (waitSec(0.5)) {
                    swingState++;
                    setTime = this.time;
                }
                break;
            case 3:
                swingPow = -MAX_POW;
                swingPos = swingMotor.getCurrentPosition();
                if (swingPos <= COUNTS_PER_REV * 0.05) {
                    swingState = 0;
                    setTime = this.time;
                }
                break;
        }
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        swingPow = Range.clip(swingPow, -MAX_POW, MAX_POW);
        swingMotor.setPower(swingPow);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("mpow", swingMotor.getPower());
        telemetry.addData("mpos", swingMotor.getCurrentPosition());
        telemetry.addData("s", swingState);
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

