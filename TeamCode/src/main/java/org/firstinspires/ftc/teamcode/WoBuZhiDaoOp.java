package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Alex on 12/12/2018.
 */


@TeleOp(name = "WoBuZhiDaoOp", group = "Default")
@Disabled
public class WoBuZhiDaoOp extends OpMode {
    //Declare any motors, servos, and sensors
    DriveTrain driveTrain = new DriveTrain(DriveMode.ARCADE, 4);
    Servo markerServo; //180 servo
    DcMotor latchMotor;
    DcMotor armMotor;
    DcMotor bodyMotor;
    Servo clawArm; //180
    DcMotor sleighMotor; //40:1 AndyMark Neverest Motor
    Servo tiltServo1; //180 Rev Servo, left side
    Servo tiltServo2; //180 Rev Servo, right side

    //Declare any variables & constants pertaining to SleighIntake
    final double SLEIGH_PWR_MAX = 0.8;
    double currentSleighPwr = 0.0;
    final double TILT_MIN = 0.0; //Sleigh is Down
    final double TILT_MAX = 0.45; //Sleigh is Up
    final double TILT_START_POS = TILT_MAX;
    double currentTiltPos = TILT_START_POS;
    boolean tiltDown = false;

    //Declare any variables & constants pertaining to Drive Train
    final double DRIVE_PWR_MAX = 0.90;

    //Declare any variables & constants pertaining to Marker
    final double MIN_MARKER_POS = 0.29; //start pos
    final double MAX_MARKER_POS = 1.00; //drop pos
    final double START_MARK_POS = MIN_MARKER_POS;
    double currentMarkPos = START_MARK_POS;

    //Declare any variables & constants pertaining to Latch System
    final double LATCH_PWR = 0.80;
    double currentLatchPwr = 0.0;
    final double MAX_LATCH_SPEED = (1.00) / 2;
    double currentLatchSpeed = 0.5;

    //Declare any variables & constants pertaining to Mineral System
    final double CLAW_ARM_START_POS = 0.5;
    final double MAX_CLAW_SPEED = (1.00) * 0.5;
    double clawArmPosition = CLAW_ARM_START_POS;
    final double CLAW_MAX = 1.0;
    final double CLAW_MIN = 0.0;
    final double ARM_PWR_MAX = 0.7;
    final double BODY_PWR_MAX = 0.8;
    double currentBodyPwr = 0.0;
    double currentArmPwr = 0.0;

    //Vuforia Stuff
    //Elapsed time and measurement constants
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Detector object
    GoldAlignDetector detector;


    public WoBuZhiDaoOp() {}

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
        //driveTrain.initializeIMU();
        //initializeDogeforia();
        //telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }
    @Override public void start() {
        runtime.reset();
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

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        //initializeMineralIntake();
        initializeSleighIntake();
        initializeDriveTrain();
        initializeMarker();
        initializeLatch();
    }

    void initializeSleighIntake() {
        currentSleighPwr = Range.clip(currentSleighPwr, -SLEIGH_PWR_MAX, SLEIGH_PWR_MAX);
        sleighMotor.setPower(currentSleighPwr);
        currentTiltPos = Range.clip(currentTiltPos, TILT_MIN, TILT_MAX);
        tiltServo1.setPosition(currentTiltPos);
        tiltServo2.setPosition(currentTiltPos);
    }

    void initializeLatch() {
        currentLatchPwr = Range.clip(currentLatchPwr, -LATCH_PWR, LATCH_PWR);
        latchMotor.setPower(currentLatchPwr);
    }

    void initializeMarker() {
        currentMarkPos = Range.clip(currentMarkPos, MIN_MARKER_POS, MAX_MARKER_POS);
        markerServo.setPosition(currentMarkPos);
    }

    void initializeDriveTrain() {
        driveTrain.initialize();
    }

    void initializeMineralIntake() {
        clawArmPosition = Range.clip(clawArmPosition,CLAW_MIN,CLAW_MAX);
        clawArm.setPosition(clawArmPosition);
        currentArmPwr = Range.clip(currentArmPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        armMotor.setPower(currentArmPwr);
        currentBodyPwr = Range.clip(currentBodyPwr,-BODY_PWR_MAX,BODY_PWR_MAX );
        bodyMotor.setPower(currentBodyPwr);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        //telemetryMineralIntake();
        telemetryDriveTrain();
        telemetryMarker();
        telemetryLatch();
        telemetrySleighIntake();
    }

    void telemetrySleighIntake() {
        telemetry.addData("SLEIGH INTAKE", "TELEMETRY");
        telemetry.addData(">>>Sleigh Pwr", sleighMotor.getPower());
        telemetry.addData(">>>Tilt1 Pos", tiltServo1.getPosition());
        telemetry.addData(">>>Tilt2 Pos", tiltServo2.getPosition());
    }

    void telemetryLatch() {
        telemetry.addData("LATCH", "TELEMETRY");
        telemetry.addData("Latch Pwr", latchMotor.getPower());
    }

    void telemetryMarker() {
        telemetry.addData("MARKER", "TELEMETRY");
        telemetry.addData("Marker Pos", markerServo.getPosition());
    }

    void telemetryDriveTrain() {
        driveTrain.telemetry();
    }

    void telemetryMineralIntake() {
        telemetry.addData("MINERAL INTAKE", "TELEMETRY");
        telemetry.addData(">>>Claw Pos",clawArm.getPosition());
        telemetry.addData(">>>Arm Pwr",armMotor.getPower());
        telemetry.addData(">>>Body Pwr",armMotor.getPower());
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        //updateMineralIntake();
        updateDriveTrain();
        updateMarker();
        updateLatch();
        updateSleighIntake();
    }

    void updateSleighIntake() {
        currentSleighPwr = -gamepad2.left_stick_y * SLEIGH_PWR_MAX;
        if (gamepad2.left_bumper && tiltDown) {
            currentTiltPos = TILT_MAX;
            tiltDown = false;
        }
        else if (gamepad2.right_bumper && !tiltDown) {
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

    void updateLatch() {
        currentLatchPwr = -gamepad2.right_stick_y * LATCH_PWR;
    }

    void updateMarker() {
        if(gamepad2.a) {
            currentMarkPos = MIN_MARKER_POS;
        }
        else if (gamepad2.y) {
            currentMarkPos = MAX_MARKER_POS;
        }
    }

    void updateDriveTrain() {
        driveTrain.update();
    }

    void updateMineralIntake() {
        updateClaw();
        updateArm();
        updateBody();
    }
    void updateClaw(){ clawArmPosition = -gamepad1.left_stick_y * MAX_CLAW_SPEED + CLAW_ARM_START_POS; }
    void updateArm(){
        currentArmPwr = -gamepad2.left_stick_y * DRIVE_PWR_MAX;
    }
    void updateBody(){
        currentBodyPwr = -gamepad2.right_stick_y * DRIVE_PWR_MAX;
    }

    //Create Methods that will update the driver data
    void initializeDogeforia() {
        // Setup camera and Vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Set Vuforia parameters
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.fillCameraMonitorViewParent = true;

        // Init Dogeforia
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Set target names
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        // Set trackables' location on field
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        //Set camera displacement
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        // Set phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        //Set info for the trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Activate targets
        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector(); // Create a gold aligndetector
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //Setup Vuforia
        vuforia.setDogeCVDetector(detector); // Set the Vuforia detector
        vuforia.enableDogeCV(); //Enable the DogeCV-Vuforia combo
        vuforia.showDebug(); // Show debug info
        vuforia.start(); // Start the detector
    }

    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {
        driveTrain.resetEncoders();
    }
    void calibrateAutoVariables() {
        driveTrain.encoderTargetReached = false;
        driveTrain.angleTargetReached = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    boolean goldAligned() {
        boolean isAligned = false;
        int centerValue = 275;
        int uncertainty = 25;
        if (Math.abs(detector.getXPosition() - centerValue) <= uncertainty)
            isAligned = true;
        return isAligned;
    }

}



