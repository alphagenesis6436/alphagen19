package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Alex on 11/18/2018.
 * This program is for the Sleigh Intake Prototype and
 * the Swing Catapult Prototype.
 */

@TeleOp(name = "SleighCatapultOp", group = "Default")
//@Disabled
public class SleighCatapultOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor sleighMotor; //40:1 AndyMark Neverest Motor
    DcMotor catapultMotor; //20:1 AndyMark Neverest Motor
    DcMotor drawerSlideMotor; //40:1 AndyMark Neverest Motor
    Servo tiltServo1; //180 Rev Servo, left side
    Servo tiltServo2; //180 Rev Servo, right side

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final int COUNTS_PER_REV40 = 1120;
    final double COUNTS_PER_REV20 = 537.6;
    final double SLEIGH_PWR_MAX = 0.8;
    double currentSleighPwr = 0.0;
    final double CATAPULT_PWR_MAX = 0.85;
    double currentCatapultPwr = 0.0;
    boolean catapultIsOn = false;
    final double DRAWERSLIDE_PWR_MAX = 0.4;
    double currentDrawerSlidePwr = 0.0;
    int currentCatapultPos = 0;
    int catapultState = 0;
    final double TILT_MIN = 0.0; //Sleigh is Down
    final double TILT_MAX = 0.45; //Sleigh is Up
    final double TILT_START_POS = TILT_MIN;
    double currentTiltPos = TILT_START_POS;
    boolean tiltDown = false;
    boolean buttonPressed = false;

    public SleighCatapultOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        sleighMotor = hardwareMap.dcMotor.get("sm");
        sleighMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        catapultMotor = hardwareMap.dcMotor.get("cm");
        catapultMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drawerSlideMotor = hardwareMap.dcMotor.get("ds");
        drawerSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize servos
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);

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
        updateDrawerSlide();
    }

    void updateCatapult() {
        if (gamepad2.y) { //button to reset catapult encoder
            DcMotor.RunMode temp = catapultMotor.getMode();
            catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            catapultMotor.setMode(temp);
        }
        switch (catapultState) {
            case 0: //wait for launch button to be pressed
                currentCatapultPwr = 0;
                catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (gamepad2.b && !gamepad2.start && currentTiltPos == TILT_MIN) { // b button launches two balls
                    catapultState++;
                    catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setTime = this.time;
                }
                else if (gamepad2.x && currentTiltPos == TILT_MIN) { // x button launches two cubes
                    catapultState--;
                    catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setTime = this.time;
                }
                else if (gamepad2.a) { // bring down catapult manually
                    currentCatapultPwr = -0.30;
                }
                break;

            //process to launch BALLS
            case 1: //launch catapult by rotating motor forward
                currentCatapultPwr = CATAPULT_PWR_MAX;
                currentCatapultPos = catapultMotor.getCurrentPosition();
                if (currentCatapultPos >= COUNTS_PER_REV20 * 0.15) {
                    catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    catapultState++;
                    setTime = this.time;
                }
                break;
            case 2: //stop catapult motor for 1/2 a second
                if (waitSec(0.5)) {
                    catapultState++;
                    catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setTime = this.time;
                }
                break;
            case 3: //bring catapult back to original position
                currentCatapultPwr = -0.4;
                if (waitSec(1)) {
                    catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    catapultState = 0;
                    setTime = this.time;
                }
                break;


                //process to launch CUBES
            //process to launch balls
            case -1: //launch catapult by rotating motor forward
                catapultMotor.setTargetPosition((int)(COUNTS_PER_REV20 * 0.5));
                catapultMotor.setPower(CATAPULT_PWR_MAX);
                if (!catapultMotor.isBusy()) {
                    catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    catapultState--;
                    setTime = this.time;
                }
                break;
            case -2: //stop catapult motor for 1/2 a second
                if (waitSec(0.5)) {
                    catapultState--;
                    catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setTime = this.time;
                }
                break;
            case -3: //bring catapult back to original position
                currentCatapultPwr = -0.4;
                if (waitSec(1)) {
                    catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    catapultState = 0;
                    setTime = this.time;
                }
                break;
        }
    }

    void updateSleighIntake() {
        currentSleighPwr = gamepad2.left_stick_y * SLEIGH_PWR_MAX;
        if (gamepad2.right_bumper && tiltDown) {
            currentTiltPos = TILT_MAX;
            tiltDown = false;
        }
        else if (gamepad2.left_bumper && !tiltDown) {
            currentTiltPos = TILT_MAX - 0.20;
            tiltDown = true;
            setTime = this.time;
        }
        if (tiltDown && waitSec(1)) {
            tiltServo1.getController().pwmEnable();
            currentTiltPos = TILT_MIN;
        }
        else if (tiltDown && waitSec(0.1)) {
            tiltServo1.getController().pwmDisable();
        }

    }
    void updateDrawerSlide() {
        currentDrawerSlidePwr = gamepad2.right_stick_y * DRAWERSLIDE_PWR_MAX;
    }

    void initialization() {
        //Clip and Initialize Sleigh Intake Motor and Servo
        currentSleighPwr = Range.clip(currentSleighPwr, -SLEIGH_PWR_MAX, SLEIGH_PWR_MAX);
        sleighMotor.setPower(currentSleighPwr);
        currentTiltPos = Range.clip(currentTiltPos, TILT_MIN, TILT_MAX);
        tiltServo1.setPosition(currentTiltPos);
        tiltServo2.setPosition(currentTiltPos);
        //Clip and Initialize Swing Catapult Motor
        currentCatapultPwr = Range.clip(currentCatapultPwr, -CATAPULT_PWR_MAX, CATAPULT_PWR_MAX);
        if (catapultMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            catapultMotor.setPower(currentCatapultPwr);
        }
        currentDrawerSlidePwr = Range.clip(currentDrawerSlidePwr, -DRAWERSLIDE_PWR_MAX, DRAWERSLIDE_PWR_MAX);
        drawerSlideMotor.setPower(currentDrawerSlidePwr);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Sleigh Pwr", sleighMotor.getPower());
        telemetry.addData("Catapult Pwr", catapultMotor.getPower());
        telemetry.addData("Catapult Enc", catapultMotor.getCurrentPosition());
        telemetry.addData("Tilt1 Pos", tiltServo1.getPosition());
        telemetry.addData("Tilt2 Pos", tiltServo2.getPosition());
        telemetry.addData("DrawerSlide Pwr", drawerSlideMotor.getPower());
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