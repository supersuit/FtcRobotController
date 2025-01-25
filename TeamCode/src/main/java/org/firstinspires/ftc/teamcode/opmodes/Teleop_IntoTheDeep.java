/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_IntoTheDeep", group="Robot")
public class Teleop_IntoTheDeep extends LinearOpMode {

    /* Declare OpMode members. */
    // ======================= MOTORS =======================
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor extensionMotor = null;
//    public DcMotor odometryX = null;
//    public DcMotor odometryY = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;

    // ======================= SERVOS =======================
    public Servo claw = null;
    double clawOffset = 0;
    public CRServo intake = null;
    public CRServo PivotServo = null;
    public Servo basketServo = null;

    // ======================= STATE MACHINES =======================
    int liftMotorStateMachine = 1;
    int clawStateMachine = 1;
    int scaleSpeedStateMachine = 1;
    double scaleTurningSpeed = .8;
    double scaleFactor = 1; //.5;
    int direction = -1;

    // ======================= SENSORS =======================
    public DistanceSensor distanceSensor;
    HardwareMap hwMap = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime currentTime = new ElapsedTime();
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ; // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double v1, v2, v3, v4;
    public CameraName Webcam1;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double liftMotorPower = 0;

        ElapsedTime liftTimer = new ElapsedTime();

        boolean rightStickButtonPushed;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        float hsvValuesFloor[] = {0F,0F,0F};
//        int colorSensorState = 0, pixels = 0;

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
//        odometryX = hardwareMap.get(DcMotor.class, "odometryX");
//        odometryY = hardwareMap.get(DcMotor.class, "odometryY");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Webcam1  = hardwareMap.get(CameraName.class, "Webcam 1");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // NO NEED FOR ENCODERS!!!
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        PivotServo = hardwareMap.get(CRServo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // Send telemetry message to signify robot waiting;

        telemetry.addLine("__ Qualifier - Teleop Code Initialized");
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        basketServo.setPosition(0);

        // convert the RGB values to HSV values.
//        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//        double DEFAULTHUE = hsvValues[0];

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentTime.reset();

            // ========================== DRIVER CONTROLLER ================================================

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // When the direction value is Freversed this if statement inverts the addition and subtraction for turning.
            // Modify scaleSpeedStateMachine to where it is only active when extension slide is out
//            switch(scaleSpeedStateMachine) {
//                case 1: {
//                    if (gamepad1.x) {
//                        scaleSpeedStateMachine = 2;
//                        scaleFactor = 0.5;
//                    }
//                }
//                break;
//                case 2: {
//                    if (!gamepad1.x) {
//                        scaleSpeedStateMachine = 3;
//                    }
//                }
//                break;
//                case 3: {
//                    if (gamepad1.x) {
//                        scaleSpeedStateMachine = 4;
//                        scaleFactor = 1;
//                    }
//                }
//                break;
//                case 4: {
//                    if (!gamepad1.x) {
//                        scaleSpeedStateMachine = 1;
//                    }
//                }
//                break;
//            }

// ===================== ASCENT MOTOR =====================
            if (gamepad1.left_bumper) {
                ascentMotor1.setPower(1);
                ascentMotor2.setPower(-1);
            } else if (gamepad1.right_bumper) {
                ascentMotor1.setPower(-1);
                ascentMotor2.setPower(1);
            } else {
                ascentMotor1.setPower(0);
                ascentMotor2.setPower(0);
            }
//            if (distanceSensor.getDistance(DistanceUnit.MM) < 30) {
//                ascentMotor1.setPower(0);
//                ascentMotor2.setPower(0);
//            }

// ===================== EXTENSION MOTOR =====================
            if (gamepad1.dpad_up) {
                // Extension motor out
                extensionMotor.setPower(0.8);
            } else if (gamepad1.dpad_down) {
                // Extension motor in
                extensionMotor.setPower(-0.8);
            } else {
                extensionMotor.setPower(0);
            }

// ===================== INTAKE SERVOS =====================
            // Outtake:
            if (gamepad1.right_trigger > 0) { //|| gamepad2.right_trigger > 0) {
                outtake();
            }
            // Intake:
            else if (gamepad1.left_trigger > 0) { //|| gamepad2.left_trigger > 0) {
                intake();
            } else {
                stopIntakeOROuttake();
            }


            double dpad_y = 0, dpad_x = 0;
            if (gamepad1.dpad_left) {
                dpad_x = -2;
            }
            if (gamepad1.dpad_right) {
                dpad_x = 2;
            }
            if (gamepad1.dpad_up) {
                dpad_y = -2;
            }
            if (gamepad1.dpad_down) {
                dpad_y = 2;
            }

            double r, robotAngle, rightX;
//            if (gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right) {
//                r = Math.hypot(-dpad_x, dpad_y);
//                robotAngle = Math.atan2(-dpad_y, dpad_x) - Math.PI / 4;
//                rightX = 0;
//                scaleFactor = 1;
//            }
//            else {
            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            scaleFactor = 1;
//                if (extensionMotor.getPower() > 0) {
//                    scaleFactor = .5;
//                }
//                else {
//                    scaleFactor = 1;
//                }
//            }

            if ((liftMotor.getCurrentPosition() > 200) || (extensionMotor.getCurrentPosition() > 300)) {
                scaleTurningSpeed = .4;
            } else
                scaleTurningSpeed = .8;


            // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            // GM0 Code for Debugging
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);


            // ========================== AUTOMATED COLLECTION SYSTEM ===========================================
            /* Pseudocode for collection system:
            1. Driver presses
             */

            // ========================== OPERATOR CONTROLLER ===========================================

// ===================== LIFTMOTOR =====================
//          FIND MOTOR TARGET POSITIONS FOR LIFTMOTOR!!!
//            if (gamepad2.left_bumper) {
//                liftMotor.setPower(1);
//            }
//            else if (gamepad2.right_bumper) {
//                liftMotor.setPower(-1);
//            }
//            else {
//                liftMotor.setPower(0);
//            }

            if (gamepad2.right_stick_y < -0.03) { //checks out
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 0.5; //.setPower(0.5);
            } else if (gamepad2.right_stick_y > 0.03) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = -0.4; //liftMotor.setPower(-0.4);
            }
            else if (gamepad2.left_stick_y < -0.03) { //checks out
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 0.8; //.setPower(0.5);
            } else if (gamepad2.left_stick_y > 0.03) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = -0.8; //liftMotor.setPower(-0.4);
            }
            else {
                if (!liftMotor.isBusy()) {
//                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotorPower = .05; //liftMotor.setPower(0.07);
                    if ((liftMotor.getCurrentPosition() <= 15)) { //added so not continuously run the motor
                        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftMotorPower = 0; //liftMotor.setPower(0);
                    }
                }
            }

            switch (liftMotorStateMachine) {
                case 1: {
                    if (gamepad2.dpad_up) { //check for first button hit
                        telemetry.addData("Right Bumper Active",1);
                        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftMotor.setTargetPosition(3846);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotorPower = .8; //liftMotor.setPower(1);
                        liftMotorStateMachine = 2;
                    }
                    break;
                }
                case 2: {
                    if (gamepad2.dpad_up == false) {
                        liftMotorStateMachine = 2;
                    }
                    break;
//                }
//                case 3: {
//                    if (gamepad2.dpad_up) {
//                        liftMotor.setTargetPosition(572);
//                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        liftMotor.setPower(1);
//                        liftMotorStateMachine = 4;
//                    }
//                    break;
//                }
//                case 4: {
//                    if (gamepad2.dpad_up == false) {
//                        liftMotorStateMachine = 5;
//                    }
//                    break;
//                }
//                case 5: {
//                    if (gamepad2.dpad_up) {
//                        liftMotor.setTargetPosition(866);
//                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        liftMotor.setPower(1);
//                        liftMotorStateMachine = 6;
//                    }
//                    break;
//                }
//                case 6: {
//                    if (gamepad2.dpad_up == false) {
//                        liftMotorStateMachine = 7;
//                    }
//                    break;
//                }
//                case 7: {
//                    if (gamepad2.dpad_up) {
//                        liftMotor.setTargetPosition(1158);
//                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        liftMotor.setPower(1);
//                        liftMotorStateMachine = 7;
//                    }
//                    break;
//                }
//                case 8: {
//                    if (gamepad2.dpad_up == false) {
//                        liftMotorStateMachine = 1;
//                    }
//                    break;
                }     //
            }
            if (gamepad2.dpad_down) {
                liftMotor.setTargetPosition(0);
                liftMotorPower = .8; //liftMotor.setPower(1);
                liftTimer.reset();
                liftMotorStateMachine = 1;
                clawStateMachine = 1;
            }

            liftMotor.setPower(liftMotorPower);
//
//// ===================== MANUAL CONTROL OF LIFTMOTOR =====================
//            if (gamepad2.right_stick_y < -0.03) { //checks out
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor.setPower(0.5);
//            }
//            else if (gamepad2.right_stick_y > 0.03) {
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor.setPower(-0.5);
//            }
//            else {
//                if (!liftMotor.isBusy()) {
//                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
//                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftMotor.setPower(0.05);
//                    if (liftMotor.getCurrentPosition() <= 15) { //added so not continuously run the motor
//                        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        liftMotor.setPower(0);
//                    }
//                }
//            }

// ===================== CLAW =====================
            // FIND CLAW SET POSITIONS!!!
            if (gamepad2.left_bumper) {
                claw.setPosition(.5);
                // Open claw
            }
            if (gamepad2.right_bumper) {
                claw.setPosition(1);
                // Close claw
            }

// ===================== PIVOT SERVO =====================
            if (gamepad1.x) {
                PivotServo.setPower(-0.8);
                // Return RP Servo
            }
            else if (gamepad1.b) {
                PivotServo.setPower(0.8);
                // Drop Pivot Servo
            }
            else {
                PivotServo.setPower(0);
                // Drop Pivot Servo
            }

// ===================== BASKET SERVO =====================
            if (gamepad2.a) {
//                basketServo.setPosition(0.34);
                basketServo.setPosition(0);
                // Return basket servo
            }
            else if (gamepad2.y) {
                basketServo.setPosition(1);
                // Dump basket servo
            }
//            else if ((gamepad2.y || gamepad2.a) && (extensionMotor.getCurrentPosition() < 200)) {
//                extensionMotor.setPower(1); // extend motor out so basket does not get hit
//                sleep(1000);
//                basketServo.setPosition(0);
//            }


//            if (gamepad2.dpad_down){
//                pixelPlacerServoStateMachine = 1; //reset the pixel placer state machine so it goes to mid on the next placement.
//                if (pixelLiftMotor.getCurrentPosition() > -700) {
////                    pixelPlacerServo.setPosition(0);
//                    //sleep(1300);
//                    pixelLiftMotor.setTargetPosition(0);
//                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    pixelLiftMotor.setPower(-.5);
//                    pixelliftMotorStateMachine = 1;
//                }
//                else {
//                    pixelLiftMotor.setTargetPosition(0);
//                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    pixelLiftMotor.setPower(-.5);
//                    pixelliftMotorStateMachine = 1;
////                    pixelPlacerServo.setPosition(0);
//                }
//            }
//
////          Click Right_stick_button to lift liftMotor to set position
//            switch (pixelliftMotorStateMachine) {
//                case 1: {
//                    if (gamepad2.dpad_up) { //check for first button hit {
//                        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        pixelLiftMotor.setTargetPosition(-484);
//                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        pixelLiftMotor.setPower(-.5);
//                        pixelliftMotorStateMachine++;
//                    }
//                    break;
//                }
//                case 2: {
//                    if (gamepad2.dpad_up == false) { //dpad button is hit again
//                        pixelliftMotorStateMachine = 3;
//                    }
//                    break;
//                }
//                case 3: {
//                    if (gamepad2.dpad_up) {
//                        pixelLiftMotor.setTargetPosition(-986);
//                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        pixelLiftMotor.setPower(-.5);
//                        // Lift your motor to set position
//                        pixelliftMotorStateMachine = 4;
//                    }
//                    break;
//                }
//                case 4: {
//                    if (gamepad2.dpad_up == false) {
//                        pixelliftMotorStateMachine = 5;
//                    }
//                    break;
//                }
//                case 5: {
//                    if (gamepad2.dpad_up) { // dpad button is hit again
//                        pixelLiftMotor.setTargetPosition(-1681);
//                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        pixelLiftMotor.setPower(-.8);
//                            // Lift your motor to set position
//                        pixelliftMotorStateMachine = 6;
//                    }
//                    break;
//                }
//                case 6: {
//                    if (gamepad2.dpad_up == false) {
//                        pixelliftMotorStateMachine = 7;
//                    }
//                    break;
//                }
//                case 7: {
//                    if (gamepad2.dpad_up) {
//                        pixelLiftMotor.setTargetPosition(-2019);
//                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        pixelLiftMotor.setPower(-.5);
//                        // Lift your motor to set position
//                    }
//                    break;
//                }
//            }


            // ==================================== TELEMETRY =========================================
            // Send telemetry message to signify robot running;
            // For configuration SET UP TELEMETRY FOR EACH ATTACHMENT !!!

            telemetry.addData("currentTime", currentTime.milliseconds());
//            // ======================= MOTORS =======================
//            telemetry.addData("leftFront: ", leftFront.getCurrentPosition());
//            telemetry.addData("leftBack: ", leftBack.getCurrentPosition());
//            telemetry.addData("rightBack: ", rightBack.getCurrentPosition());
//            telemetry.addData("rightFront: ", rightFront.getCurrentPosition());
//            telemetry.addData("v1", v1);
//            telemetry.addData("v2", v2);
//
//            telemetry.addData("v3", v3);
//            telemetry.addData("v4", v4);
//            telemetry.addData("liftMotor pos: ", liftMotor.getCurrentPosition());
//            telemetry.addData("liftMotor spd: ", liftMotor.getPower());
//            telemetry.addData("extensionMotor ", extensionMotor.getCurrentPosition());
//            telemetry.addData("odometryX", ascentMotor1.getCurrentPosition());
//            telemetry.addData("odometryY", ascentMotor2.getCurrentPosition());
//
//            // ======================= SERVOS =======================
//            telemetry.addData("intake: ", intake.getPower());
//            telemetry.addData("PivotServo: ", PivotServo.getPower());
//            telemetry.addData("basketServo: ", basketServo.getPosition());
//            telemetry.addData("claw: ", claw.getPosition());
//            telemetry.addData("distanceSensor -->", distanceSensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("ascentMotor1 -->", ascentMotor1.getPower());
//            telemetry.addData("ascentMotor2 -->", ascentMotor2.getPower());

            // ======================= TESTING =======================
//            telemetry.addData("liftTimer: ", liftTimer.time(TimeUnit.SECONDS));
//            telemetry.addData("odometryX", odometryX.getCurrentPosition());
//            telemetry.addData("odometryY", odometryY.getCurrentPosition());

            telemetry.update();
//            sleep(50);
        }
    }
    public void liftMotor_Claw_Reset () {
        claw.setPosition(0); // leave claw open
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void liftMotor_basketServo_Reset () {
        basketServo.setPosition(0); // return basket
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void basket_place_sequence () {
        extensionMotor.setTargetPosition(85); // find the set position for the 1 inch adjustment
        basketServo.setPosition(1); // dump basket
        sleep(500);
        basketServo.setPosition(0); // pull back basket
        extensionMotor.setTargetPosition(0); // return extension motor
    }
    public void intake () {
        intake.setPower(-1);
    }
    public void outtake () {
        intake.setPower(1);
    }
    public void stopIntakeOROuttake () {
        intake.setPower(0);
    }
}


/* PseudoCode TeleOp

// NOTE: Updated pseudocode is in programming logs!!

1.Drive:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

2.Grab Pixels:
-Intake Pixels

3.Transport Pixels:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

4.Place Pixels:
-Slide plate using linear slide
-Drop pixels onto backdrop

 */