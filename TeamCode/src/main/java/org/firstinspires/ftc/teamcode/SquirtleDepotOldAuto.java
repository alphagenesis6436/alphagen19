package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GoldPosition.CENTER;
import static org.firstinspires.ftc.teamcode.GoldPosition.LEFT;
import static org.firstinspires.ftc.teamcode.GoldPosition.NOT_FOUND;
import static org.firstinspires.ftc.teamcode.GoldPosition.RIGHT;

/**
 * Updated by Alex on 2/10/2019.
 */
@Autonomous(name = "SquirtleDepotOldAuto", group = "default")
@Disabled
public class SquirtleDepotOldAuto extends SquirtleOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;
    String goldPos = "NOT FOUND";
    boolean latchReset = false;

    public SquirtleDepotOldAuto() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveTrain.syncOpMode(gamepad1, telemetry, hardwareMap);
        driveTrain.setDrivePwrMax(DRIVE_PWR_MAX);
        driveTrain.setMotors();
        telemetry.addData(">", "Drive Train Initialization Successful");

        latchMotor = hardwareMap.dcMotor.get("lm");
        telemetry.addData(">", "Latch Initialization Successful");

        scoringMotor = hardwareMap.dcMotor.get("sm");
        scoringMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Scoring Mechanism Initialization Successful");

        intakeMotor = hardwareMap.dcMotor.get("im");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Intake Initialization Successful");

        extenderMotor1 = hardwareMap.dcMotor.get("em1");
        extenderMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotor2 = hardwareMap.dcMotor.get("em2");
        extenderMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Extender Initialization Successful");

        driveTrain.initializeIMU();
        initializeDogeforia();
        telemetry.addData(">", "Vuforia Initialization Successful");

        // Make sure that the sound files exist on the phone
        bandSoundID = hardwareMap.appContext.getResources().getIdentifier("band", "raw", hardwareMap.appContext.getPackageName());
        marchSoundID   = hardwareMap.appContext.getResources().getIdentifier("march",   "raw", hardwareMap.appContext.getPackageName());
        weowSoundID   = hardwareMap.appContext.getResources().getIdentifier("weow",   "raw", hardwareMap.appContext.getPackageName());
        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (bandSoundID != 0)
            bandFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, bandSoundID);

        if (marchSoundID != 0)
            marchFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, marchSoundID);
        if (weowSoundID != 0)
            weowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, weowSoundID);

        // Display sound status
        telemetry.addData("band sound",   bandFound ?   "Found" : "NOT found\n Add band.mp3 to /src/main/res/raw" );
        telemetry.addData("march sound", marchFound ? "Found" : "NOT found\n Add march.mp3 to /src/main/res/raw"  );
        telemetry.addData("WEOW sound", weowFound ? "Found" : "NOT found\n Add weow.mp3 to /src/main/res/raw"  );

        telemetry.addData(">", "Press Start to continue");
    }

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateGoal, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));
        telemetry.addData("Gold Position", goldPos);
        switch (goldPosition) {
            case NOT_FOUND: goldPos = "NOT FOUND";
                break;
            case LEFT: goldPos = "LEFT";
                break;
            case CENTER: goldPos = "CENTER";
                break;
            case RIGHT: goldPos = "RIGHT";
                break;
        }
        telemetry.addLine();

        //retract latch
        if (state > 4) {
            if (!latchReset) {
                extendLatch(-LATCH_PWR, 0);
            }
            else {
                latchMotor.setPower(0);
            }
            if (latchValueReached) {
                latchReset = true;
            }
        }

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateGoal = "Initial Calibration";
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                scoringMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                scoringMotor.setPower(0);
                latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateGoal = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendLatch(LATCH_PWR, 19.4);

                if (latchValueReached) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    state++;
                }
                break;

            case 4:
                stateGoal = "Turn to have hook unlatch - Turn 15 cw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolute(15);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, weowSoundID);
                    telemetry.addData("Playing", "WEEOOOOWWW");
                    state++;
                }
                break;

            case 6:
                stateGoal = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(-0.9, -1.1);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 8:
                stateGoal = "Turn to original angle";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolutePID(0);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 10:
                stateGoal = "Move Toward Sampling Field - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();

                driveTrain.moveForward(-0.9, -0.4);
                extendIntake(0.9);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 12:
                stateGoal = "Extend Intake";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendIntake(0.9);

                if (waitSec(1.1)) { //Use a boolean value that reads true when state goal is completed
                    extendIntake(0);
                    state++;
                }
                break;

            case 14:
                stateGoal = "Drop Marker - Rotate intake down";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendIntake(0);
                if (waitSec(0.55)) {
                    intakeMotor.setPower(0.90);
                }
                setTiltServos(TILT_MIN);


                if (waitSec(1.25)) { //Use a boolean value that reads true when state goal is completed
                   state++;
                }
                break;

            case 16:
                stateGoal = "Rotate intake up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                setTiltServos(TILT_MAX);
                extendIntake(-0.4);

                if (waitSec(0.05)) { //Use a boolean value that reads true when state goal is completed
                    intakeMotor.setPower(0);
                    state++;
                }
                break;

            case 18:
                stateGoal = "Retract Intake";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                extendIntake(-0.8);
                driveTrain.moveForward(0.9, 0.30); //AND back up

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    extendIntake(0);
                    state++;
                }
                break;

            /*case 12:
                stateGoal = "Move Toward Lander - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(0.9, 0.5);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;*/

            case 20:
                stateGoal = "Turn to have phone face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolutePID(-90);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 22: //Search for Gold Mineral by Driving Backward and using GoldMineralDetector
                stateGoal = "Scan for Gold Mineral - Drive Backward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(-0.25, -1.75);

                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    goldPosition = (Math.abs(driveTrain.getRevolutionsDriven()) <= 1) ? CENTER : LEFT;
                    state += 3; //skip alignment step for gold in right position
                }
                else if (driveTrain.encoderTargetReached) { //if gold not found within 1.5 revolutions, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = RIGHT;
                    state++;
                }
                break;


            case 24: //Drive Forward to face gold cube
                stateGoal = "Align to Cube in Right Position - Drive Forward (SKIP UNLESS RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(0.90, 1.65);
                }
                else {
                    driveTrain.moveForward(0, 0);
                }


                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;


            case 26:
                stateGoal = "Turn to have front of robot face sampling field - Turn 90 cw (30 ccw if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                int targetAngle = (goldPosition == RIGHT)? -140 : 0;
                telemetry.addData("TargetAngle", targetAngle);
                driveTrain.turnAbsolutePID(targetAngle);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 28: //Knock off cube - Drive Forward
                stateGoal = "Knock off cube - Drive Backward (Forward if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(0.90, 1.0);
                }
                else {
                    driveTrain.moveForward(-0.90, -0.9);
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 30: //Knock off cube - Drive Forward
                stateGoal = "Knock off cube - Drive Forward (Backward if RIGHT)";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (goldPosition == RIGHT) {
                    driveTrain.moveForward(-0.90, -1.2);
                }
                else {
                    driveTrain.moveForward(0.90, 0.6);
                }
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 32:
                stateGoal = "Turn to have phone face sampling field - Turn 90 ccw";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnAbsolutePID(-95);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;


            case 34:
                stateGoal = "Drive Backward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantPower();
                double targetTime = (goldPosition == LEFT)? 3 : 3.5;
                driveTrain.moveForward(-0.9);
                extendIntake(waitSec(1) ? 0 : 0.4);
                setTiltServos(TILT_SCORE);


                if (waitSec(targetTime)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }


            case 1000: //Run When Autonomous is Complete
                stateGoal = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateGoal = "Calibrating";
                calibrateAutoVariables();
                resetEncoders();
                if (waitSec(0.01)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}