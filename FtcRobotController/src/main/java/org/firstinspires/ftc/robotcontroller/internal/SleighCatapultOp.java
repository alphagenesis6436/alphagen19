package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

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
 *          1 is fully clockwise
 *          Most Servos we have are broken and cannot operate in the full range
 *      360 Degree (Continuous) HiTechnic Servos
 *          0 is max speed in clockwise direction
 *          0.5 is speed = 0, servo stops
 *          1 is max speed in counterclockwise direction
 *      Rev Servos
 *
 *
 */


@TeleOp(name = "SleighCatapultOp", group = "Default")
//@Disabled
public class SleighCatapultOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor sleighMotor; //40:1 AndyMark Neverest Motor
    DcMotor catapultMotor; //40:1 AndyMark Neverest Motor
    Servo tiltServo; //180 Rev Servo

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final int COUNTS_PER_REV = 1120;
    final double SLEIGH_PWR_MAX = 0.8;
    double currentSleighPwr = 0.0;
    final double CATAPULT_PWR_MAX = 0.8;
    double currentCatapultPwr = 0.0;
    int currentCatapultPos = 0;
    int catapultState = 0;
    final double TILT_MIN = 0.0;
    final double TILT_MAX = 1.0;
    final double TILT_START_POS = 0.5;
    double currentTiltPos = TILT_START_POS;

    public SleighCatapultOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        sleighMotor = hardwareMap.dcMotor.get("sm");
        sleighMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        catapultMotor = hardwareMap.dcMotor.get("cm");
        catapultMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize servos
        tiltServo = hardwareMap.servo.get("ts");

        //Initialize sensors

        telemetry.addData(">", "Press Start to continue");
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
        updateSleighIntake();
        updateCatapult();
    }

    void updateCatapult() {
        switch (catapultState) {
            case 0: //wait for launch button to be pressed
                currentCatapultPwr = 0;
                if (gamepad2.a) {
                    catapultState++;
                    setTime = this.time;
                }
                break;
            case 1: //launch catapult by rotating motor forward
                currentCatapultPwr = CATAPULT_PWR_MAX;
                currentCatapultPos = catapultMotor.getCurrentPosition();
                if (currentCatapultPos >= COUNTS_PER_REV * 0.8) {
                    catapultState++;
                    setTime = this.time;
                }
                break;
            case 2: //stop catapult motor for 1/2 a second
                currentCatapultPwr = 0;
                if (waitSec(0.5)) {
                    catapultState++;
                    setTime = this.time;
                }
                break;
            case 3: //bring catapult back to original position
                currentCatapultPwr = -CATAPULT_PWR_MAX;
                currentCatapultPos = catapultMotor.getCurrentPosition();
                if (currentCatapultPos <= COUNTS_PER_REV * 0.05) {
                    catapultState = 0;
                    setTime = this.time;
                }
                break;
        }
    }

    void updateSleighIntake() {
        currentSleighPwr = gamepad2.left_stick_y * SLEIGH_PWR_MAX;
        if (gamepad2.right_bumper) {
            currentTiltPos = TILT_MAX;
        }
        else if (gamepad2.left_bumper) {
            currentTiltPos = TILT_MIN;
        }
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        currentSleighPwr = Range.clip(currentSleighPwr, -SLEIGH_PWR_MAX, SLEIGH_PWR_MAX);
        sleighMotor.setPower(currentSleighPwr);
        currentCatapultPwr = Range.clip(currentCatapultPwr, -CATAPULT_PWR_MAX, CATAPULT_PWR_MAX);
        catapultMotor.setPower(currentCatapultPwr);
        currentTiltPos = Range.clip(currentTiltPos, TILT_MIN, TILT_MAX);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Sleigh Pwr", sleighMotor.getPower());
        telemetry.addData("Catapult Pwr", catapultMotor.getPower());
        telemetry.addData("Tilt Pos", tiltServo.getPosition());
    }


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
    void calibrateAutoVariables() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}