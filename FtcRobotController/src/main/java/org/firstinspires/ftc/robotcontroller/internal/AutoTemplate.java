package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Updated by Alex Arteaga on 2/11/2019.
 *
 * Purpose:     This program is meant to be a template/guide for creating any autonomous program
 *              for a SPECIFIED TeleOp program. Programmers should copy and paste this program with
 *              a new name that fits your autonomous strategy. The new autonomous MUST extend off
 *              of the TeleOp of the robot being used.
 *
 * Style:       This template takes a State Machine approach to programming the robot to perform
 *              step by step actions in a linear fashion. Utilizing a switch statement with the
 *              input, state, the autonomous program will progress through various user-created
 *              states meant to have the robot complete actions to score points OR setup for TeleOp.
 *              Each state is composed of two parts: (1) State Goal & (2) Robot Action.
 *
 * state:       This is an int type variable that comes with every TeleOp. Its initial value will
 *              always be 0. This variable will be incremented (state++) at the completion of
 *              every state. Exception is when your autonomous is designed with multiple paths. In
 *              that case, increment by an odd number (state += 3) to skip over unused states.
 *
 * State Goal:  The State Goal is a simple task of which the robot must complete in order to move
 *              through your overall autonomous strategy. For example, if your overall autonomous
 *              strategy is to have the robot drive in a path outlining a 24" square, the first
 *              State Goal would simply be "Move Forward 24 Inches". The second State Goal would be
 *              "Turn Clockwise 90 degrees". The next State Goals would be exactly the same as these
 *              in a way that all the State Goals performed in order will accomplish the overall
 *              autonomous strategy. In code, the State Goal is represented by a String stateGoal
 *              which should be renamed at the beginning of each case. In order for the robot to
 *              progress to the next state, each case should end with an if-statement with an
 *              appropriate conditional that matches the State Goal. For example, in the first state
 *              of driving in a square, the code would be:
 *
 *                          if (driveTrain.encoderTargetReached) {
 *                              advanceState();
 *                          }
 *
 * Robot        The Robot Action is the actual code used to have the robot actively reach the State
 * Action:      Goal. For example, when the State Goal is "Move Forward 24 Inches", the code for the
 *              Robot Action would be:
 *
 *                          driveTrain.moveForward(0.90, 24);
 *
 *              Even though each state should have only one State Goal, they can have as many Robot
 *              Actions the user finds necessary; however the norm is just one Robot Action.
 *
 * State 0:     Each autonomous will start at State 0. This state is reserved for calibrating all
 *              the sensors, setting servos to their initial positions, and ensuring all motors have
 *              0 power.
 *
 * Default      This state is called in between each user-created state and is used to calibrate any
 * State:       autonomous variables necessary such as resetting the encoders, the encoder variables
 *              and angle variables. The State Goal is "Calibrating" and will end after 0.01 seconds
 *              have passed. This amount of time can be changed at the user's discretion.
 *
 * State 1000:  This state is reserved to officially end the autonomous. Similar to State 0, this
 *              state is reserved for calibrating all the sensors, setting servos to their initial
 *              positions, and ensuring all motors have 0 power.
 */
@Autonomous(name = "AutoTemplate", group = "default")
@Disabled
public class AutoTemplate extends TeleOpMode {
    //Declare and Initialize any variables needed for this specific autonomous program


    public AutoTemplate() {}

    @Override
    public void init() {

    }

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData("" + state, stateGoal);
        telemetry.addData("State Time", String.format("%.1f", this.time - setTime));

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateGoal = "Initial Calibration";
                calibrateAutoVariables();
                advanceState();
                break;

            case 2: //Describe the Robot’s Goal & Actions in this state
                stateGoal = "First State Goal";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)


                if (true) { //Use a boolean value that reads true when state goal is completed
                    advanceState();
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateGoal = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateAutoVariables();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateGoal = "Calibrating";
                calibrateAutoVariables();
                if (waitSec(0.01)) {
                    advanceState();
                }
                break;
        }
    }

    @Override
    void telemetry() {

    }

    @Override
    void updateData() {

    }

    //Create any methods needed for this specific autonomous program

}