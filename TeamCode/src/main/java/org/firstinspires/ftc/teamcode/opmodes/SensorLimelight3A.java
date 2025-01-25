 /*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//=====================================================================================
//* * * * * * * * * * * * * * * CURRENT AUTONOMOUS CODE * * * * * * * * * * * * * * * *
//=====================================================================================

 package Opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobot;

import java.util.List;
import java.util.function.ToDoubleFunction;

 /*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@Autonomous(name = "Sensor: Limelight3A", group = "Sensor")
public class SensorLimelight3A extends LinearOpMode {

    public SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    public Limelight3A limelight;
    public double expectedStartPoseX = 0;
    public double expectedStartPoseY = 0;
    boolean runonce = false;
    boolean prelimelight = false;
    // ======================= MOTORS =======================
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;
    public DcMotor extensionMotor  = null;
//    public DcMotor odometryX = null;
//    public DcMotor odometryY = null;

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
    double scaleFactor = .5;
    int direction = -1;

    private static SampleAuto_IntoTheDeep.START_POSITION startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT;
    enum START_POSITION {
        //IN RELATION TO SUBMERSIBLE
        RED_BLUE_RIGHT,
        RED_BLUE_LEFT,
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(true);

        // Define and Initialize Motors/Sensors/Servos
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
//        odometryX = hardwareMap.get(DcMotor.class, "odometryX");
//        odometryY = hardwareMap.get(DcMotor.class, "odometryY");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        PivotServo = hardwareMap.get(CRServo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.addData("Robot Starting Position", "X: %.2f", "Y: %.2f");
        telemetry.update();
        selectStartingPosition();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
//        limelight.start();
//        limelight.pipelineSwitch(0);

        while (opModeIsActive() && runonce == false) {
            if ((startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) || (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) && prelimelight == false) {
                telemetry.addData("Running >> ", startPosition.getClass());
                telemetry.update();
/*                the expected pose is the final position of the robot
//                expectedStartPoseX = -12.6;//26.8;
//                expectedStartPoseY = -62.7;
//                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
//                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
//                double driveDistance = -24 + DeltaY;
//                double strafeDistance = 26 + DeltaX;

                // Pseudocode for BlueLeft/RedLeft auto program:
                // 1. Close claw on specimen
                // 2. Drive up to the submersible
                // 3. Turn 180 degrees
                // 4. Place specimen
                // 5. Turn 180 degrees
                // 6. Strafe towards april tag 16
                // 7. Intake sample
                // 8. Turn right 45 degrees
                // 9. Drive backwards (if necessary) to line up against the basket
                // 10. Place sample in the basket
                // 11. Park in Level 1 Ascent (back of the robot is touching the low rung) */
/*                // TESTING ATTACHMENTS
//                // ======================= claw servo
//                claw.setPosition(1); // close claw
//                // ======================= liftmotor
//                liftMotor.setTargetPosition(1846);
//                setLiftMotorClaw();
//                liftMotor.setTargetPosition(3864);
//                setLiftMotorBasket();
//                liftMotor.setTargetPosition(0);
//                setLift0MotorDown();
//                // ======================= extension, basket, and pivot motor/servo
//                intake();
//                // ======================= basket servo
//                basketDumpSequence(); */
                // START BACKWARDS
                // ======================= DRIVE
                claw.setPosition(1); // close claw
                sleep(250);
                liftMotor.setTargetPosition(2000); // lift liftmotor
                setLiftMotorClaw();
                robot.drive( -22, 0.45, 0.25);
                robot.drive( -7, 0.3, 0.25);
                // ======================= PLACE SPECIMEN
                liftMotor.setTargetPosition(0); // pull down liftmotor
                setLiftMotorDown();
                sleep(250);
                claw.setPosition(0.5); // open claw
//                // ======================= DRIVE
                robot.drive(17, 0.5, 0.25);
                robot.turnTo(180,0.5,0.5);
                robot.resetHeading(); // reset heading for limelight
                // ======================= START LIMELIGHT
                /* Starts polling for data.  If you neglect to call start(), getLatestResult() will return null. */
                sleep(200);
                limelight.start();
                limelight.pipelineSwitch(0);
                telemetry.addData("limelight started", "");
//                // =======================
////                robot.drive(-4, 0.4, 0.25); // Drive backwards
////                liftMotor_Claw_Reset();
////                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
////                robot.strafe(26, 0.45, 0.5); // strafe left 26 inches
////                PivotServo.setPower(0.8); // drop pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                intake(); // intake MIDDLE sample
////                PivotServo.setPower(-0.8); // return pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                stopIntakeOROuttake(); // Stop intake
////                robot.drive(-4, 0.4, 0.25);// Drive backward
////                robot.turnTo(-45, 0.45, 0.5); // turn right 45 degrees
////                robot.drive(-12, 0.4, 0.25); // drive backwards 12 inches
////                liftMotor.setTargetPosition(1158); // Lift claw/basket slide
////                basket_place_sequence(); // Flip basket servo
////                liftMotor_basketServo_Reset();
////                robot.turnTo(45, 0.45, 0.5);// turn left 45 degrees
////                robot.strafe(-24, 0.4, 0.25);// strafe right
////                robot.drive(27, 0.4, 0.25); // drive forward 27 inches
////                robot.turnTo(90, 0.4, 0.25); // turn left 90 degrees
////                robot.drive(-24, 0.4, 0.25); // drive backward 24 inches
////                robot.strafe(50,0.5,0.25); // strafe right and park
////
////                claw.setPosition(1); // close claw on specimen
//////                robot.drive(50,0.5,0.5);
////                robot.strafe(25,0.5,0.5);
////                sleep(500);
//
////                robot.strafe(12,0.60,0.5);
//
////                telemetry.addData("odometryX", odometryX.getCurrentPosition());
////                telemetry.addData("odometryY", odometryY.getCurrentPosition());
////                telemetry.addData("liftMotor pos: ", liftMotor.getCurrentPosition());
//                telemetry.update();
                runonce = true;
                prelimelight = true;
                sleep(3000);
            }
//            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT && prelimelight == false) {
//                telemetry.addData("Running RED_BLUE_RIGHT ", "");
//                telemetry.update();
//                // the expected pose is the final position of the robot
//                // for this specific test, the final position = (starting position + 24 in)
//                expectedStartPoseX = -12.6;//26.8;
//                expectedStartPoseY = -62.7;
////                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
////                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
////                double driveDistance = -24 + DeltaY;
////                double strafeDistance = -36 + DeltaX;
//
//                // Pseudocode for BlueRight/RedRight auto program:
//                // [NOTE you start facing backwards]
//                // 1. Close claw on specimen
//                // 2. Drive up to the submersible
//                // 3. Place specimen
//                // 4. Strafe towards april tag 14
//                // 5. Turn 180 degrees
//                // 6. Pick up sample
//                // 7. Place sample(s) in observation zone
//                // 8. Drive backwards to park in the observation zone
//
////                robot.drive(-24, 0.4, 0.25); // drive BACKWARDS 24 inches
////                liftMotor.setTargetPosition(572); // Lift claw/basket slide
////                robot.drive(4, 0.4, 0.25);// Move forward
////                liftMotor.setTargetPosition(276); // Pull down
////                claw.setPosition(0); // Open claw
////                robot.drive(4, 0.4, 0.25);// Move forward
////                liftMotor_Claw_Reset();
////                robot.strafe(-36, 0.45, 0.5); // strafe right towards tag 14
////                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
////                robot.drive(-22, 0.4, 0.25); // drive backwards 22 inches
////                PivotServo.setPower(0.8); // drop pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                intake(); // intake MIDDLE sample
////                PivotServo.setPower(-0.8); // return pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                stopIntakeOROuttake(); // Stop intake
////                robot.drive(-4, 0.4, 0.25); // Drive backward
////                liftMotor.setTargetPosition(276); // Lift claw/basket slide to set position
////                basket_place_sequence(); // Flip basket servo
////                liftMotor_basketServo_Reset();
////                // NOTE: You may have to repeat the previous steps to intake and deposit all of the samples
////                robot.drive(-20, 0.4, 0.25); // Drive backwards to park
//
////                robot.strafe(-50,0.5,0.25); // strafe left and park
////                claw.setPosition(1); // close claw on specimen
////                robot.drive(-50,0.5,0.5);
////                sleep(500);
////                runonce = true;
///*                telemetry.addData("odometryX", odometryX.getCurrentPosition());
////                telemetry.addData("odometryY", odometryY.getCurrentPosition());
////                telemetry.addData("odometryY", robot.driveEncoder);
//                  telemetry.addData("odometryX", robot.strafeEncoder);
// */
//            }
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose_MT2();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
//                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("txnc", result.getTxNC());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("tync", result.getTyNC());
                    telemetry.addData("Botpose (robot's orientation/location):", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT && prelimelight == true) {
                                // the expected pose is the final position of the robot
                                // for this specific test, the final position = (starting position + 24 in)
                                expectedStartPoseX = -12.6;//26.8;
                                expectedStartPoseY = -62.7;
                                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
                                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
                                double driveDistance = -24 + DeltaY;
                                double strafeDistance = 34 + DeltaX;

//                                telemetry.addData("preset_expectedStartPoseX -->", expectedStartPoseX);
//                                telemetry.addData("preset_expectedStartPoseY -->", expectedStartPoseY);
//                                telemetry.addData("DeltaX -->", DeltaX);
//                                telemetry.addData("DeltaY -->", DeltaY);
//                                telemetry.addData("driveDistance -->", driveDistance);
//                                telemetry.addData("strafeDistance -->", strafeDistance);
//                                telemetry.addData("Limelight_RobotPoseX -->", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("Limelight_RobotPoseY -->", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.update();
//                                sleep(10000);
                                // ========================================== STRAFE TO APRILTAG 16
                                // NOTE: You may have to check the startpose to see if this strafing will work
                                robot.strafe(strafeDistance, 0.55, 0.25);
                                // ======================= GRAB AND PLACE
                                robot.drive(-3.3,0.5,0.25);
                                PivotServo.setPower(-0.8);
                                sleep(500);

                                // TODO: 1/17/2025 test and fix pivot/wrist servo in the "intake()" method;
                                //  it's not pulling the intake rollers up every time
                                intake(); // grab sample - make sure you are lined up PERFECTLY
                                robot.drive(3.3,0.5,0.25);
                                robot.turnTo(-45, 0.55, 0.5);// turn right 45 degrees
                                robot.drive(-9.5,0.5,0.25);
                                liftMotor.setTargetPosition(3800);
                                setLiftMotorBasket(); // lift liftmotor
                                basketDumpSequence(); // dump basket
                                liftMotor.setTargetPosition(0);
                                setLiftMotorDown(); // return liftmotor
                                robot.turnTo(0, 0.55, 0.5);// turn right 45 degrees
                                // ^^NOTE: you may repeat the previous steps to get the rest of the samples^^

//                                robot.strafe(4,0.5,0.25);
//                                robot.drive(4,0.5,0.25);
//                                PivotServo.setPower(-0.8);
//                                sleep(500);
//                                intake(); // grab sample - make sure you are lined up PERFECTLY
//                                robot.drive(-4,0.5,0.5);
//                                robot.turnTo(-45, 0.5, 0.5);// turn right 45 degrees
//                                liftMotor.setTargetPosition(3800);
//                                setLiftMotorBasket(); // lift liftmotor
//                                basketDumpSequence(); // dump basket
//                                liftMotor.setTargetPosition(0);
//                                setLiftMotorDown(); // return liftmotor
//                                robot.turnT(0, 0.55, 0.5);// turn right 45 degrees
                                // ======================= PARK
                                robot.strafe(-20, 0.5, 0.25); // strafe right 24 inches
                                robot.drive(48, 0.5, 0.25); // drive forward 48 inches
                                robot.turnTo(90,0.55,0.25);
                                basketServo.setPosition(1);
                                robot.drive(-12, 0.5, 0.25); // strafe right 12 inches
                                robot.drive(-6,0.3,0.5);
                                sleep(500);
                                runonce = true;
                            }
/*                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                                telemetry.addData("RobotXPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("RobotYPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.addData("Yaw", fr.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES));
//                                telemetry.addData("Pitch", fr.getRobotPoseFieldSpace().getOrientation().getPitch(AngleUnit.DEGREES));
//                                telemetry.addData("Roll", fr.getRobotPoseFieldSpace().getOrientation().getRoll(AngleUnit.DEGREES));
//                                telemetry.addData("DeltaX", DeltaX);
//                                telemetry.addData("DeltaY", DeltaY);
//                                telemetry.addData("driveDistance", driveDistance);
//                                telemetry.addData("strafeDistance", strafeDistance);
//                                telemetry.addData("odometryX", ascentMotor1);
//                                telemetry.addData("odometryY", ascentMotor2); */

                            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT && prelimelight == true) {
                                // the expected pose is the final position of the robot
                                // for this specific test, the final position = (starting position + 24 in)
//                                expectedStartPoseX = -12.6;//26.8;
//                                expectedStartPoseY = -62.7;
//                                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
//                                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
//                                double driveDistance = -24 + DeltaY;
//                                double strafeDistance = -44 + DeltaX;
///*                                // Pseudocode for BlueRight/RedRight auto program:
////                                // [NOTE you start facing backwards]
////                                // 1. Close claw on specimen
////                                // 2. Drive up to the submersible
////                                // 3. Place specimen
////                                // 4. Strafe towards april tag 14
////                                // 5. Turn 180 degrees
////                                // 6. Pick up sample
////                                // 7. Place sample(s) in observation zone
////                                // 8. Drive backwards to park in the observation zone */
//                                // ========================================== STRAFE TO APRILTAG 14
//                                robot.strafe(strafeDistance,0.5,0.5);
//                                // ======================= GRAB AND PLACE
//                                intake(); // grab sample - make sure you are lined up PERFECTLY
//                                setLiftMotorBasket(); // lift liftmotor
//                                basketDumpSequence(); // dump basket
//                                setLiftMotorDown(); // return liftmotor
//                                // ^^NOTE: you may repeat the previous steps to get the rest of the samples^^
//                                // ======================= PARK
//                                robot.drive(-12,0.4,0.25);
//                                sleep(500);
//                                runonce = true;

                                // ============= Contingency plan !!
                                // Facing forwards
                                robot.drive(14,0.5,0.5);
                                robot.strafe(-45, 0.55, 0.25);
                                sleep(500);
                                runonce = true;
/*                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                                telemetry.addData("RobotXPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("RobotYPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.addData("Yaw", fr.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES));
//                                telemetry.addData("Pitch", fr.getRobotPoseFieldSpace().getOrientation().getPitch(AngleUnit.DEGREES));
//                                telemetry.addData("Roll", fr.getRobotPoseFieldSpace().getOrientation().getRoll(AngleUnit.DEGREES));
//                                telemetry.addData("DeltaX", DeltaX);
//                                telemetry.addData("DeltaY", DeltaY);
//                                telemetry.addData("driveDistance", driveDistance);
//                                telemetry.addData("strafeDistance", strafeDistance);
//                                telemetry.addData("odometryX", ascentMotor1);
//                                telemetry.addData("odometryY", ascentMotor2); */
                            }
                        }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }

//                    List<ColorResult> colorTargets = result.getColorResults();
//                    for (ColorResult colorTarget : colorTargets) {
//                        double x = detection.getTargetXDegrees(); // Where it is (left-right)
//                        double y = detection.getTargetYDegrees(); // Where it is (up-down)
//                        double area = colorTarget.getTargetArea(); // size (0-100)
//                        telemetry.addData("Color Target", "takes up " + area + "% of the image");
//                    }
                    /* Pseudocode for robot repositioning based off of apriltag:

                    deltax = pose.x - expectedPose.x;
                    deltay = pose.y - expectedPose.y;

                     // drive forward 24 inches, but the robot is setup 1 inch off the wall.
                     driveForward(24 - deltay);
                     */
                limelight.stop();
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }
    }

    public void selectStartingPosition () {
        boolean selected = false;
        while (!isStopRequested() || (selected)) {
            telemetry.addLine("State Tournament - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
//            telemetry.addData("    Specimen Testing ", "(^)");
//            telemetry.addData("    Square Testing ", "(v)");
            telemetry.addData("    Red/Blue Left    ", "(<)");
            telemetry.addData("    Red/Blue Right  ", "(>)");

//            if (gamepad1.dpad_left || gamepad2.dpad_up) {
//                startPosition = SampleAuto_IntoTheDeep.START_POSITION.SPECIMEN_TESTING;
//                selected = true;
//                break;
//            }
//            if (gamepad1.dpad_left || gamepad2.dpad_down) {
//                startPosition = SampleAuto_IntoTheDeep.START_POSITION.SQUARE_TESTING;
//                selected = true;
//                break;
//            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT;
                telemetry.addData("Red/Blue Left: ", "selected");
                telemetry.update();
                selected = true;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT;
                selected = true;
                telemetry.addData("Red/Blue Right: ", "selected");
                telemetry.update();
                break;
            }
            telemetry.update();
        }
    }
    public void setLiftMotorClaw () {
        double liftMotorPower = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = .8;
        liftMotor.setPower(liftMotorPower);
        while (liftMotor.getCurrentPosition() < 2000){
        }
        liftMotor.setPower(.05);
        sleep(1500);
    }
    public void setLiftMotorBasket () {
        double liftMotorPower = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = .8;
        liftMotor.setPower(liftMotorPower);
        while (liftMotor.getCurrentPosition() < 3800){
        }
        liftMotor.setPower(.05);
        sleep(1500);
    }
    public void setLiftMotorDown () {
        double liftMotorPower = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = .8;
        liftMotor.setPower(liftMotorPower);
        while (liftMotor.getCurrentPosition() > 500){
        }
        liftMotor.setPower(.05);
        sleep(1500);
    }
    public void liftMotor_Claw_Reset () {
        claw.setPosition(0); // leave claw open
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void liftMotor_basketServo_Reset () {
        basketServo.setPosition(0); // return basket
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void basketDumpSequence () {
        basketServo.setPosition(1); // dump basket
        sleep(1500); // this provides time for the basket to dump
        basketServo.setPosition(0); // return basket
        sleep(1500); // this provides time for the basket to return
    }
    public void intake () {
        boolean runonce = false;
        // ExtensionMotor out
        extensionMotor.setPower(1); // extensionmotor out
        sleep(500); // make sure extension motor extends all the way
        extensionMotor.setPower(0); // return extension motor

        // PivotServo drop
        PivotServo.setPower(0.8); // drop rack&pinion servo
        sleep(300); // make sure the servo drops all the way
        intake.setPower(-0.7); // start intake
        sleep(1000); // intake until
        while ((intake.getPower() == -1) && (runonce == false)) {
            PivotServo.setPower(-0.8); // drop rack&pinion servo
            sleep(500);
            runonce = true;
        }

        // Transfer sample to basket
        extensionMotor.setPower(-1); // extensionmotor out
        sleep(500); // make sure extension motor extends all the way
        extensionMotor.setPower(0); // return extension motor
        intake.setPower(0); // stop intake - make sure sample is in intake
        PivotServo.setPower(-0.8); // return rack&pinion servo
        sleep(2000);
        while ((PivotServo.getPower() == -0.8) && (runonce == false)) {
            sleep(2000); // wait for extension motor
            // NOTE: this might affect the previous while loop - which is why I implemented the runonce
            intake.setPower(-0.7);
            sleep(1000);
            runonce = true;
        }
        sleep(500);
        PivotServo.setPower(0); // stop PivotServo
        intake.setPower(-0.7);
        sleep(500);
        intake.setPower(0);
    }
    public void outtake () {
        intake.setPower(1);
    }
    public void stopIntake () {
        intake.setPower(0);
    }
}