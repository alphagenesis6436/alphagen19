package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.OpModeTemplate;

import static org.firstinspires.ftc.teamcode.GoldPosition.*;

/**
 * Updated by Alex on 11/5/2017.
 */
@Autonomous(name = "RedDepotAuto", group = "default")
@Disabled
public class RedDepotAuto extends WoBuZhiDaoOp {
    //Declare and Initialize any variables needed for this specific autonomous program
    GoldPosition goldPosition = NOT_FOUND;

    public RedDepotAuto() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveTrain.syncOpMode(gamepad1, telemetry, hardwareMap);
        driveTrain.setDrivePwrMax(DRIVE_PWR_MAX);
        driveTrain.setMotors();
        telemetry.addData(">", "Drive Train Initialization Successful");
        //armMotor = hardwareMap.dcMotor.get("arm");
        //armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //bodyMotor = hardwareMap.dcMotor.get("body");
        //bodyMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData(">", "Mineral Intake Initialization Successful");
        //Initialize servos
        //clawArm = hardwareMap.servo.get("ca");
        //Initialize Sensors
        sleighMotor = hardwareMap.dcMotor.get("sm");
        sleighMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Sleigh Intake Initialization Successful");
        driveTrain.initializeIMU();
        initializeDogeforia();
        telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                markerServo.setPosition(MAX_MARKER_POS);
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Lower Robot to Ground - Latch Pwr Up";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                latchMotor.setPower(LATCH_PWR);

                if (waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    latchMotor.setPower(0);
                    state = 1000;
                }
                break;

            case 4:
                stateName = "Unlatch Hook From Lander - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(0.4, 0.5);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

                case 6: //Search for Gold Mineral by Turning CCW and using GoldMineralDetector
                stateName = "Scan for Gold Mineral - Turn CCW";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.turnClockwise(-0.10);


                if (goldAligned()) { //if gold found, then it's either center or left position
                    driveTrain.stopDriveMotors();
                    if (Math.abs(driveTrain.getHeading()) <= 15) {
                        goldPosition = CENTER;
                    }
                    else {
                        goldPosition = LEFT;
                    }
                    state = 1000; //skip alignment step for gold in right position
                }
                else if (driveTrain.getHeading() < -25) { //if gold not found within 75 degree ccw turn, then cube is right position
                    driveTrain.stopDriveMotors();
                    goldPosition = RIGHT;
                    state = 1000;
                }
                break;


            case 8: //Turn Clockwise to face gold cube
                stateName = "Align to Cube in Right Position - Turn CW";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                if (driveTrain.getHeading() < 0) {
                    driveTrain.turnClockwise(0.80); //turn quickly until heading is 25 degrees cw of initial position
                }
                else {
                    driveTrain.turnClockwise(0.10);
                }

                if (goldAligned()) {
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                else if (driveTrain.getHeading() > 40) { //if gold not found, STOP at 75 degrees cw of initial position
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 10: //Knock off cube - Drive Forward
                stateName = "Knock off cube - Drive Forward";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                double targetRevolutions = 0;
                switch (goldPosition) {
                    case LEFT:
                    case RIGHT: targetRevolutions = 3.75;
                        break;
                    case CENTER: targetRevolutions = 3.25;
                        break;
                }
                driveTrain.moveForward(0.80, targetRevolutions);
                if (driveTrain.encoderTargetReached) {
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                calibrateAutoVariables();
                resetEncoders();
                if (waitSec(1)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}