package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.DeathBoxOp;

/**
 * Updated by Alex on 11/5/2017.
 */
@Autonomous(name = "DefaultAuto", group = "default")
@Disabled
public class DefaultAuto extends WoBuZhiDaoOp {
    //Declare and Initialize any variables needed for this specific autonomous program


    public DefaultAuto() {}

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
        latchMotor = hardwareMap.dcMotor.get("lm");
        markerServo = hardwareMap.servo.get("ms");
        sleighMotor = hardwareMap.dcMotor.get("sm");
        sleighMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        tiltServo1 = hardwareMap.servo.get("ts1");
        tiltServo1.setDirection(Servo.Direction.FORWARD);
        tiltServo2 = hardwareMap.servo.get("ts2");
        tiltServo2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Sleigh Intake Initialization Successful");
        driveTrain.initializeIMU();
        driveTrain.startIMU();
        //initializeDogeforia();
        //telemetry.addData(">", "Vuforia Initialization Successful");

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
                markerServo.setPosition(START_MARK_POS);
                tiltServo1.setPosition(TILT_START_POS);
                tiltServo2.setPosition(TILT_START_POS);
                calibrateAutoVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Drive Forward to Depot";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.8, 4.8);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 4:
                stateName = "Rotate 135 degrees clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnClockwise(135);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 6:
                stateName = "Drive Backward to Wall";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(-0.8, -0.75);

                if (driveTrain.encoderTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 8:
                stateName = "Drop Marker";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                markerServo.setPosition(MAX_MARKER_POS);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    markerServo.setPosition(MIN_MARKER_POS);
                    state++;
                }
                break;

            case 10:
                stateName = "Drive Forward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.8, 1.8);

                if (driveTrain.encoderTargetReached || waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 12:
                stateName = "Rotate 135 degrees clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnClockwise(100);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 14:
                stateName = "Drive Forward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.moveForward(0.8, 1.1);

                if (driveTrain.encoderTargetReached || waitSec(5)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 16:
                stateName = "Rotate 135 degrees clockwise to be parallel with the walls";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.turnClockwise(135);

                if (driveTrain.angleTargetReached) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state++;
                }
                break;

            case 18:
                stateName = "Drive Forward Park in Crater";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                driveTrain.runConstantSpeed();
                driveTrain.moveForward(0.8);

                if (waitSec(4.5)) { //Use a boolean value that reads true when state goal is completed
                    driveTrain.stopDriveMotors();
                    state = 1000;
                }

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                driveTrain.stopDriveMotors();
                calibrateAutoVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
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