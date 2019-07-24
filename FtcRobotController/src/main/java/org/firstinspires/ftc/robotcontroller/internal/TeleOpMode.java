package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.Mechanism;

import java.util.ArrayList;

/**
 * Created by Alex on 6/1/2017.
 * Updated by Alex on 9/30/2018
 */

/**
 * Notes about Hardware Devices
 *  DCMotors
 *      Power Range: [-1. 1]
 *      AndyMark Neverest Motors
 *          FORWARD direction follows the RIGHT HAND RULE
 *             meaning, stick out your right thumb (motor axle) and curl hand in direction of fingers
 *             (direction motor axle will spin)
 *      Tetrix Motors
 *          FORWARD direction follows the LEFT HAND RULE
 *      Rev Motors
 *          FORWARD direction is indicated on the motor!
 *  Servos
 *      Position Range: [0, 1]
 *      180 Degree HiTechnic Servos
 *          0 is fully clockwise
 *          1 is fully counter clockwise
 *          Most Servos we have are broken and cannot operate in the full range
 *      360 Degree (Continuous) HiTechnic Servos
 *          0 is max speed in clockwise direction
 *          0.5 is speed = 0, servo stops
 *          1 is max speed in counterclockwise direction
 *      Rev Servos
 *          Can act as either 180 Degree Servo or 360 Degree
 *
 *
 */


@TeleOp(name = "OpMode Template", group = "Default")
@Disabled
public abstract class TeleOpMode extends OpMode {
    //Declare any motors, servos, and sensors

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)


    public TeleOpMode() {}

    @Override public void loop() {
        //Update all the data based on driver input & set to hardware devices
        updateData();

        //Show the Real Values of the Data Using Telemetry
        telemetry();
    }

    //Show Data for Specific Robot Mechanisms
    abstract void telemetry();

    //Add in update methods for specific robot mechanisms
    abstract void updateData();

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
    String stateGoal = ""; //Overwrite this as the specific step used in Autonomous

    void advanceState() {
        state++;
        setTime = this.time;
    }

    void advanceState(int skipState) {
        state += (skipState * 2) + 1;
        setTime = this.time;
    }


    void calibrateAutoVariables() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}



