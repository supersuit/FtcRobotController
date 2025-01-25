/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobot;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

//@Autonomous(name="SampleAuto_IntoTheDeep", group = "Mr. Phil")
public class SampleAuto_IntoTheDeep extends LinearOpMode {
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private static START_POSITION startPosition = START_POSITION.RED_BLUE_RIGHT; //WHERE WE ARE ON THE FIELD/ RED CLOSE ETC

    enum START_POSITION {
        //IN RELATION TO SUBMERSIBLE
        RED_BLUE_RIGHT,
        RED_BLUE_LEFT,
        RED_BLUE_RIGHTSTRAFE,
        SPECIMEN_TESTING,
        SQUARE_TESTING,
    }

    @Override
    public void runOpMode() {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();

        robot.resetHeading();  // Reset heading to set a baseline for Auto

        selectStartingPosition();

        // Run Auto if stop was not pressed.
        if (opModeIsActive()) {

// Square Testing:
                if (startPosition == START_POSITION.SQUARE_TESTING) {
                telemetry.addData("Running SQUARE_TESTING ", "");
                telemetry.update();

                robot.drive(  20, 0.50, 0.25);
                robot.turnTo(90, 0.45, 0.5);
                robot.drive(  20, 0.50, 0.25);
                robot.turnTo(180, 0.45, 0.5);
                robot.drive(  20, 0.50, 0.25);
                robot.turnTo(270, 0.45, 0.5);
                robot.drive(  20, 0.50, 0.25);
                robot.turnTo(0, 0.45, 0.5);

                sleep(500);

    //          Drive the path again without turning.
                robot.drive(  20, 0.50, 0.15);
                robot.strafe( 20, 0.50, 0.15);
                robot.drive( -20, 0.50, 0.15);
                robot.strafe(-20, 0.50, 0.15);

                sleep(500);
                }

                if (startPosition == START_POSITION.SPECIMEN_TESTING) {
                telemetry.addData("Running SPECIMEN_TESTING ", "");
                telemetry.update();
// Specimen Placing Testing:
                { // ============== Specimen Placing Testing ==============
//              NOTE: Claw is already preloaded with specimen:
//                ClawServo.setPosition(-1); // grab specimen

                    // Set robot position
//                robot.drive(12, 0.60, 0.25); // drive forward
//                robot.strafe(-12, 0.60, 0.15); // strafe right
//                sleep(500);

//            1. Lift linear slide to (set position)
                    // Set position options: -484, -986, -1681, 2019
//                pixelLiftMotor.setTargetPosition(-1681);
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);

//            2. Move forward (set number of inches)
//                robot.drive(12, 0.60, 0.25); // drive to the chamber

//            3. Bring down linear slide to (set position - enough to where the specimen clicks in)
//            pixelLiftMotor.setTargetPosition(__);
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
//                // pull linear slide down to snap the specimen (at specified position)

//            4. Open claw and bring down linear slide
//                ClawServo.setPosition(0); // release specimen
//                pixelLiftMotor.setTargetPosition(-481);
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
//========================================== ^Specimen Placing Testing^ ==========================================
                }
            }

                if (startPosition == START_POSITION.RED_BLUE_LEFT) {
                    telemetry.addData("Running RED_BLUE_LEFT ", "");
                    telemetry.update();
// RedLeft and BlueLeft:
                    // ============== RedLeft and BlueLeft ==============
//              NOTE: Claw is already preloaded with specimen:
//                ClawServo.setPosition(-1); // grab specimen

                    // ===== SET UP TO PLACE =====
                    robot.drive(-24, 0.5, 0.25);
//                pixelLiftMotor.setTargetPosition(-1681); // raise slide
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
                    robot.drive(12, 0.5, 0.25);
                    robot.strafe(-36,0.45, 0.5);
                    robot.turnTo(-45,0.45, 0.5);
                    sleep(500);

                    // ===== PLACE SPECIMEN =====
//            pixelLiftMotor.setTargetPosition(__); // snap specimen on chamber
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
//                // pull linear slide down to snap the specimen (at specified position)
//                ClawServo.setPosition(0); // release specimen
//                pixelLiftMotor.setTargetPosition(-481); // pull down slide
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);

                    // ===== SET UP TO PLACE =====
//                robot.strafe(60, 0.60, 0.15); // strafe left
                    // Intake here // grab sample

                    // ===== GRAB SAMPLE AND PLACE =====
                /* Currently, the programming team is not sure
                 how placing will be done with AprilTag detection */
                    // This will be repeated twice to grab and place both samples in the basket

                    // ===== PARK IN LEVEL 1 ASCENT =====
//                robot.drive(24, 0.60, 0.25); // drive forward
//                robot.strafe(30, 0.60, 0.15); // strafe left
//                robot.drive(12, 0.60, 0.25); // drive forward
//                sleep(500); // park in Level 1 Ascent
                    // NOTE: for Level 1 Ascent the robot should be TOUCHING the rung

//========================================== ^RedLeft and BlueLeft^ ==========================================
                }

                if (startPosition == START_POSITION.RED_BLUE_RIGHT) {
                    telemetry.addData("Running RED_BLUE_RIGHT ", "");
                    telemetry.update();
// RedRight and BlueRight
                    // ============== RedRight and BlueRight ==============
//              NOTE: Claw is already preloaded with specimen:
//                ClawServo.setPosition(-1); // grab specimen

                    // ===== SET UP TO PLACE =====
                    robot.drive(12, 0.60, 0.25); // drive forward
                    robot.strafe(12, 0.60, 0.15); // strafe left
                    sleep(500);
//                pixelLiftMotor.setTargetPosition(-1681); // raise slide
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
                    robot.drive(12, 0.60, 0.25); // drive to the chamber

                    // ===== PLACE SPECIMEN =====
//                pixelLiftMotor.setTargetPosition(__); // snap specimen on chamber
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);
                    // pull linear slide down to snap the specimen (at specified position)
//                ClawServo.setPosition(0); // release specimen
//                pixelLiftMotor.setTargetPosition(-481); // pull down slide
//                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pixelLiftMotor.setPower(-.8);

                    // ===== SET UP TO PLACE =====
//                robot.strafe(-60, 0.60, 0.15); // strafe left
                    // Intake here // grab sample

                    // ===== GRAB SAMPLE AND PLACE =====
                /* Currently, the programming team is not sure
                 how placing will be done with AprilTag detection */
                    // This will be repeated twice to grab and place both samples in the basket

                    // ===== PARK IN OBSERVATION ZONE =====
//                robot.drive(24, 0.60, 0.25); // drive forward
//                robot.strafe(30, 0.60, 0.15); // strafe left
//                robot.drive(12, 0.60, 0.25); // drive forward
//                sleep(500); // park in Level 1 Ascent
                    // NOTE: for Level 1 Ascent the robot should be TOUCHING the rung

//========================================== ^RedLeft and BlueLeft^ ==========================================
                }

            }
        }

        public void selectStartingPosition () {
            boolean selected = false;
            while (!isStopRequested() || (selected)) {
                telemetry.addLine("State Tournament - Code Initialized");
                telemetry.addData("---------------------------------------", "");
                telemetry.addLine("Select Starting Position using DPAD Keys");
                telemetry.addData("    Specimen Testing ", "(^)");
                telemetry.addData("    Square Testing ", "(v)");
                telemetry.addData("    Red/Blue Left    ", "(<)");
                telemetry.addData("    Red/Blue Right  ", "(>)");

                if (gamepad1.dpad_left || gamepad1.dpad_up) {
                    startPosition = START_POSITION.SPECIMEN_TESTING;
                    selected = true;
                    break;
                }
                if (gamepad1.dpad_left || gamepad1.dpad_down) {
                    startPosition = START_POSITION.SQUARE_TESTING;
                    selected = true;
                    break;
                }
                if (gamepad1.dpad_left || gamepad1.dpad_left) {
                    startPosition = START_POSITION.RED_BLUE_LEFT;
                    selected = true;
                    break;
                }
                if (gamepad1.dpad_right || gamepad1.dpad_right) {
                    startPosition = START_POSITION.RED_BLUE_RIGHT;
                    selected = true;
                    break;
                }
                telemetry.update();
            }
        }
    }