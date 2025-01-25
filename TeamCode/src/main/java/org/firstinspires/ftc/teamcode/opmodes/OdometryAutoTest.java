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

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
 import com.qualcomm.hardware.limelightvision.LLResult;
 import com.qualcomm.hardware.limelightvision.LLResultTypes;
 import com.qualcomm.hardware.limelightvision.LLStatus;
 import com.qualcomm.hardware.limelightvision.Limelight3A;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobot;

 import java.util.List;

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
 @Autonomous(name = "OdometryAutoTest", group = "Sensor")
 public class OdometryAutoTest extends LinearOpMode {

     public SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
     private Limelight3A limelight;
     public double expectedStartPoseX = 0;
     public double expectedStartPoseY = 0;
     boolean runonce = false;
     // ======================= MOTORS =======================
     public DcMotor leftFront   = null;
     public DcMotor leftBack   = null;
     public DcMotor  rightFront  = null;
     public DcMotor  rightBack  = null;
     public DcMotor liftMotor = null;
 //    public DcMotor ascentMotor1 = null;
 //    public DcMotor ascentMotor2 = null;
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
     private double          headingError  = 0;

     // These variable are declared here (as class members) so they can be updated in various methods,
     // but still be displayed by sendTelemetry()
     private double  targetHeading = 0;
     private double  driveSpeed    = 0;
     private double  turnSpeed     = 0;
     private double  leftSpeed     = 0;
     private double  rightSpeed    = 0;
     private int     leftTarget    = 0;
     private int     rightTarget   = 0;


     // Calculate the COUNTS_PER_INCH for your specific drive train.
     // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     // This is gearing DOWN for less speed and more torque.
     // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
     static final double     COUNTS_PER_MOTOR_REV    = 145.1 ;   // eg: GoBILDA 312 RPM Yellow Jacket
     static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
     static final double     WHEEL_DIAMETER_INCHES   = 1.889765 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
             (WHEEL_DIAMETER_INCHES * 3.1415);

     // These constants define the desired driving/control characteristics
     // They can/should be tweaked to suit the specific robot drive train.
     static final double     DRIVE_SPEED             = 0.3;     // Max driving speed for better distance accuracy.
     static double     TURN_SPEED              = 0.3;     // Max Turn speed to limit turn rate
     static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
     // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
     // Define the Proportional control coefficient (or GAIN) for "heading control".
     // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
     // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
     // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
     static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
     static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
     double gyroAngle = 0;
     BNO055IMU imu;

     private static SampleAuto_IntoTheDeep.START_POSITION startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT;
     enum START_POSITION {
         //IN RELATION TO SUBMERSIBLE
         RED_BLUE_RIGHT,
         RED_BLUE_LEFT,
     }
     @Override
     public void runOpMode() throws InterruptedException {

         robot.initialize(true);
         initGyro();

         // Define and Initialize Motors/Sensors/Servos
 //        limelight = hardwareMap.get(Limelight3A.class, "limelight");
         leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
         rightBack = hardwareMap.get(DcMotor.class, "rightBack");
         rightFront = hardwareMap.get(DcMotor.class, "rightFront");
         leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
         liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
         extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
 //        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
 //        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
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

         // Only use these if necessary for configuration:
 //        ascentMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 //        ascentMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         telemetry.setMsTransmissionInterval(11);

 //        limelight.pipelineSwitch(0);

         /* Starts polling for data.  If you neglect to call start(), getLatestResult() will return null. */
 //        limelight.start();

         telemetry.addData(">", "Robot Ready.  Press Play.");
 //        telemetry.addData("Robot Starting Position", "X: %.2f", "Y: %.2f");
         telemetry.update();
         selectStartingPosition();

         waitForStart();

         robot.resetHeading();  // Reset heading to set a baseline for Auto

         while (opModeIsActive()) {
             if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) {
                 telemetry.addData("Running RED_BLUE_LEFT ", "");
                 telemetry.update();
                 // the expected pose is the final position of the robot
                 // for this specific test, the final position = (starting position + 24 in)
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
                 // 11. Park in Level 1 Ascent (back of the robot is touching the low rung)

                 // this gamepad1.dpad_up condition is for testing
                 // it just gives you a moment to see what limelight is seeing:
                 claw.setPosition(1); // close claw on specimen
 //                robot.drive(-24, 0.4, 0.25); // drive backward 24 inches
 //                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
 //                liftMotor.setTargetPosition(572); // Lift claw/basket slide
 //                robot.drive(4, 0.4, 0.25);// Move forward
 //                liftMotor.setTargetPosition(276); // Pull down
 //                claw.setPosition(0); // Open claw
 //                robot.drive(-4, 0.4, 0.25); // Drive backwards
 //                liftMotor_Claw_Reset();
 //                robot.strafe(26, 0.45, 0.5); // strafe left 26 inches
 //                PivotServo.setPower(0.8); // drop pivot servo
 //                sleep(200); // run pivot servo for __ seconds
 //                PivotServo.setPower(0); // stop pivot servo if necessary
 //                intake(); // intake MIDDLE sample
 //                PivotServo.setPower(-0.8); // return pivot servo
 //                sleep(200); // run pivot servo for __ seconds
 //                PivotServo.setPower(0); // stop pivot servo if necessary
 //                stopIntakeOROuttake(); // Stop intake
 //                robot.drive(-4, 0.4, 0.25);// Drive backward
 //                robot.turnTo(-45, 0.45, 0.5); // turn right 45 degrees
 //                robot.drive(-12, 0.4, 0.25); // drive backwards 12 inches
 //                liftMotor.setTargetPosition(1158); // Lift claw/basket slide
 //                basket_place_sequence(); // Flip basket servo
 //                liftMotor_basketServo_Reset();
 //                robot.turnTo(45, 0.45, 0.5);// turn left 45 degrees
 //                robot.strafe(-24, 0.4, 0.25);// strafe right
 //                robot.drive(27, 0.4, 0.25); // drive forward 27 inches
 //                robot.turnTo(90, 0.4, 0.25); // turn left 90 degrees
 //                robot.drive(-24, 0.4, 0.25); // drive backward 24 inches
 //                robot.strafe(50,0.5,0.25); // strafe right and park
                 robot.drive(50,0.5,0.5);
                 sleep(500);
                 runonce = true;

 //                telemetry.addData("odometryX", odometryX);
 //                telemetry.addData("odometryY", odometryY);
             }

             if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) {
                 telemetry.addData("Running RED_BLUE_RIGHT ", "");
                 telemetry.update();
                 // the expected pose is the final position of the robot
                 // for this specific test, the final position = (starting position + 24 in)
                 expectedStartPoseX = -12.6;//26.8;
                 expectedStartPoseY = -62.7;
 //                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
 //                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
 //                double driveDistance = -24 + DeltaY;
 //                double strafeDistance = -36 + DeltaX;

                 // Pseudocode for BlueRight/RedRight auto program:
                 // [NOTE you start facing backwards]
                 // 1. Close claw on specimen
                 // 2. Drive up to the submersible
                 // 3. Place specimen
                 // 4. Strafe towards april tag 14
                 // 5. Turn 180 degrees
                 // 6. Pick up sample
                 // 7. Place sample(s) in observation zone
                 // 8. Drive backwards to park in the observation zone

                 claw.setPosition(1); // close claw on specimen
 //                robot.drive(-24, 0.4, 0.25); // drive BACKWARDS 24 inches
 //                liftMotor.setTargetPosition(572); // Lift claw/basket slide
 //                robot.drive(4, 0.4, 0.25);// Move forward
 //                liftMotor.setTargetPosition(276); // Pull down
 //                claw.setPosition(0); // Open claw
 //                robot.drive(4, 0.4, 0.25);// Move forward
 //                liftMotor_Claw_Reset();
 //                robot.strafe(-36, 0.45, 0.5); // strafe right towards tag 14
 //                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
 //                robot.drive(-22, 0.4, 0.25); // drive backwards 22 inches
 //                PivotServo.setPower(0.8); // drop pivot servo
 //                sleep(200); // run pivot servo for __ seconds
 //                PivotServo.setPower(0); // stop pivot servo if necessary
 //                intake(); // intake MIDDLE sample
 //                PivotServo.setPower(-0.8); // return pivot servo
 //                sleep(200); // run pivot servo for __ seconds
 //                PivotServo.setPower(0); // stop pivot servo if necessary
 //                stopIntakeOROuttake(); // Stop intake
 //                robot.drive(-4, 0.4, 0.25); // Drive backward
 //                liftMotor.setTargetPosition(276); // Lift claw/basket slide to set position
 //                basket_place_sequence(); // Flip basket servo
 //                liftMotor_basketServo_Reset();
 //                // NOTE: You may have to repeat the previous steps to intake and deposit all of the samples
 //                robot.drive(-20, 0.4, 0.25); // Drive backwards to park

 //                robot.strafe(-50,0.5,0.25); // strafe left and park
                 robot.drive(-50,0.5,0.5);
                 sleep(500);
                 runonce = true;

 //                telemetry.addData("odometryX", odometryX);
 //                telemetry.addData("odometryY", odometryY);

             LLStatus status = limelight.getStatus();
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

 //                            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) {
 //                                telemetry.addData("Running RED_BLUE_LEFT ", "");
 //                                telemetry.update();
 //                                // the expected pose is the final position of the robot
 //                                // for this specific test, the final position = (starting position + 24 in)
 //                                expectedStartPoseX = -12.6;//26.8;
 //                                expectedStartPoseY = -62.7;
 //                                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
 //                                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
 //                                double driveDistance = -24 + DeltaY;
 //                                double strafeDistance = 26 + DeltaX;
 //
 //                                // Pseudocode for BlueLeft/RedLeft auto program:
 //                                // 1. Close claw on specimen
 //                                // 2. Drive up to the submersible
 //                                // 3. Turn 180 degrees
 //                                // 4. Place specimen
 //                                // 5. Turn 180 degrees
 //                                // 6. Strafe towards april tag 16
 //                                // 7. Intake sample
 //                                // 8. Turn right 45 degrees
 //                                // 9. Drive backwards (if necessary) to line up against the basket
 //                                // 10. Place sample in the basket
 //                                // 11. Park in Level 1 Ascent (back of the robot is touching the low rung)
 //
 //                                // this gamepad1.dpad_up condition is for testing
 //                                // it just gives you a moment to see what limelight is seeing:
 //                                claw.setPosition(0); // close claw on specimen
 //                                robot.drive(driveDistance, 0.4, 0.25); // drive backward 24 inches
 //                                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
 //                                liftMotor.setTargetPosition(572); // Lift claw/basket slide
 //                                robot.drive(4, 0.4, 0.25);// Move forward
 //                                liftMotor.setTargetPosition(276); // Pull down
 //                                claw.setPosition(0); // Open claw
 //                                robot.drive(-4, 0.4, 0.25); // Drive backwards
 //                                liftMotor_Claw_Reset();
 //                                robot.strafe(strafeDistance, 0.45, 0.5); // strafe left 26 inches
 //                                PivotServo.setPower(0.8); // drop pivot servo
 //                                sleep(200); // run pivot servo for __ seconds
 //                                PivotServo.setPower(0); // stop pivot servo if necessary
 //                                intake(); // intake MIDDLE sample
 //                                PivotServo.setPower(-0.8); // return pivot servo
 //                                sleep(200); // run pivot servo for __ seconds
 //                                PivotServo.setPower(0); // stop pivot servo if necessary
 //                                stopIntakeOROuttake(); // Stop intake
 //                                robot.drive(-4, 0.4, 0.25);// Drive backward
 //                                robot.turnTo(-45, 0.45, 0.5); // turn right 45 degrees
 //                                robot.drive(-12, 0.4, 0.25); // drive backwards 12 inches
 //                                liftMotor.setTargetPosition(1158); // Lift claw/basket slide
 //                                basket_place_sequence(); // Flip basket servo
 //                                liftMotor_basketServo_Reset();
 //                                robot.turnTo(45, 0.45, 0.5);// turn left 45 degrees
 //                                robot.strafe(-24, 0.4, 0.25);// strafe right
 //                                robot.drive(27, 0.4, 0.25); // drive forward 27 inches
 //                                robot.turnTo(90, 0.4, 0.25); // turn left 90 degrees
 //                                robot.drive(-24, 0.4, 0.25); // drive backward 24 inches
 //                                sleep(500);
 //                                runonce = true;
 //
 //                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
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
 //                                telemetry.addData("odometryY", ascentMotor2);
 //                            }
 //
 //                            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) {
 //                                telemetry.addData("Running RED_BLUE_RIGHT ", "");
 //                                telemetry.update();
 //                                // the expected pose is the final position of the robot
 //                                // for this specific test, the final position = (starting position + 24 in)
 //                                expectedStartPoseX = -12.6;//26.8;
 //                                expectedStartPoseY = -62.7;
 //                                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
 //                                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
 //                                double driveDistance = -24 + DeltaY;
 //                                double strafeDistance = -36 + DeltaX;
 //
 //                                // Pseudocode for BlueRight/RedRight auto program:
 //                                // [NOTE you start facing backwards]
 //                                // 1. Close claw on specimen
 //                                // 2. Drive up to the submersible
 //                                // 3. Place specimen
 //                                // 4. Strafe towards april tag 14
 //                                // 5. Turn 180 degrees
 //                                // 6. Pick up sample
 //                                // 7. Place sample(s) in observation zone
 //                                // 8. Drive backwards to park in the observation zone
 //
 //                                claw.setPosition(0); // close claw on specimen
 //                                robot.drive(driveDistance, 0.4, 0.25); // drive BACKWARDS 24 inches
 //                                liftMotor.setTargetPosition(572); // Lift claw/basket slide
 //                                robot.drive(4, 0.4, 0.25);// Move forward
 //                                liftMotor.setTargetPosition(276); // Pull down
 //                                claw.setPosition(0); // Open claw
 //                                robot.drive(4, 0.4, 0.25);// Move forward
 //                                liftMotor_Claw_Reset();
 //                                robot.strafe(strafeDistance, 0.45, 0.5); // strafe right towards tag 14
 //                                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
 //                                robot.drive(-22, 0.4, 0.25); // drive backwards 22 inches
 //                                PivotServo.setPower(0.8); // drop pivot servo
 //                                sleep(200); // run pivot servo for __ seconds
 //                                PivotServo.setPower(0); // stop pivot servo if necessary
 //                                intake(); // intake MIDDLE sample
 //                                PivotServo.setPower(-0.8); // return pivot servo
 //                                sleep(200); // run pivot servo for __ seconds
 //                                PivotServo.setPower(0); // stop pivot servo if necessary
 //                                stopIntakeOROuttake(); // Stop intake
 //                                robot.drive(-4, 0.4, 0.25); // Drive backward
 //                                liftMotor.setTargetPosition(276); // Lift claw/basket slide to set position
 //                                basket_place_sequence(); // Flip basket servo
 //                                liftMotor_basketServo_Reset();
 //                                // NOTE: You may have to repeat the previous steps to intake and deposit all of the samples
 //                                robot.drive(-20, 0.4, 0.25); // Drive backwards to park
 //                                sleep(500);
 //                                runonce = true;
 //
 //                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
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
 //                                telemetry.addData("odometryY", ascentMotor2);

                                 // ======================= TESTING =======================
 //                                telemetry.addData("odometryX", odometryX);
 //                                telemetry.addData("odometryY", odometryY);
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

                 }
             } else {
                 telemetry.addData("Limelight", "No data available");
             }
         }
         limelight.stop();
     }
     /*
      * ====================================================================================================
      * Driving "Helper" functions are below this line.
      * These provide the high and low level methods that handle driving straight and turning.
      * ====================================================================================================
      */

     // **********  HIGH Level driving functions.  ********************

     /**
      *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
      *  Move will stop if either of these conditions occur:
      *  1) Move gets to the desired position
      *  2) Driver stops the OpMode running.
      *
      * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
      * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
      * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *                   If a relative angle is required, add/subtract from the current robotHeading.
      */
     public void driveStraight(double maxDriveSpeed,
                               double distance,
                               double heading) {

         // Ensure that the OpMode is still active
         if (opModeIsActive()) {

             // Determine new target position, and pass to motor controller
             int moveCounts = (int)(distance * COUNTS_PER_INCH);
             leftTarget = leftFront.getCurrentPosition() + moveCounts;
             rightTarget = rightFront.getCurrentPosition() + moveCounts;

             // Set Target FIRST, then turn on RUN_TO_POSITION
             leftFront.setTargetPosition(leftTarget);
             leftBack.setTargetPosition(leftTarget);
             rightFront.setTargetPosition(rightTarget);
             rightBack.setTargetPosition(rightTarget);

             leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             // Set the required driving speed  (must be positive for RUN_TO_POSITION)
             // Start driving straight, and then enter the control loop
             maxDriveSpeed = Math.abs(maxDriveSpeed);
             moveRobot(maxDriveSpeed, 0);

             // keep looping while we are still active, and BOTH motors are running.
             while (opModeIsActive() &&
                     (leftFront.isBusy() && rightFront.isBusy())) {

                 // Determine required steering to keep on heading
                 turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                 // if driving in reverse, the motor correction also needs to be reversed
                 if (distance < 0)
                     turnSpeed *= -1.0;

                 // Apply the turning correction to the current driving speed.
                 moveRobot(driveSpeed, turnSpeed);

                 // Display drive status for the driver.
 //                sendTelemetry(true);
             }

             // Stop all motion & Turn off RUN_TO_POSITION
             moveRobot(0, 0);
             leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }
     }

     /**
      *  Spin on the central axis to point in a new direction.
      *  <p>
      *  Move will stop if either of these conditions occur:
      *  <p>
      *  1) Move gets to the heading (angle)
      *  <p>
      *  2) Driver stops the OpMode running.
      *
      * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
      * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *              If a relative angle is required, add/subtract from current heading.
      */
     public void turnToHeading(double maxTurnSpeed, double heading) {

         // Run getSteeringCorrection() once to pre-calculate the current error
         getSteeringCorrection(heading, P_DRIVE_GAIN);

         // keep looping while we are still active, and not on heading.
         while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

             // Determine required steering to keep on heading
             turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

             // Clip the speed to the maximum permitted value.
             turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

             // Pivot in place by applying the turning correction
             moveRobot(0, turnSpeed);

             // Display drive status for the driver.
 //            sendTelemetry(false);
         }

         // Stop all motion;
         moveRobot(0, 0);
     }

     /**
      *  Obtain & hold a heading for a finite amount of time
      *  <p>
      *  Move will stop once the requested time has elapsed
      *  <p>
      *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
      *
      * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
      * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *                   If a relative angle is required, add/subtract from current heading.
      * @param holdTime   Length of time (in seconds) to hold the specified heading.
      */
     public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

         ElapsedTime holdTimer = new ElapsedTime();
         holdTimer.reset();

         // keep looping while we have time remaining.
         while (opModeIsActive() && (holdTimer.time() < holdTime)) {
             // Determine required steering to keep on heading
             turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

             // Clip the speed to the maximum permitted value.
             turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

             // Pivot in place by applying the turning correction
             moveRobot(0, turnSpeed);

             // Display drive status for the driver.
 //            sendTelemetry(false);
         }

         // Stop all motion;
         moveRobot(0, 0);
     }

     // **********  LOW Level driving functions.  ********************

     /**
      * Use a Proportional Controller to determine how much steering correction is required.
      *
      * @param desiredHeading        The desired absolute heading (relative to last heading reset)
      * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
      * @return                      Turning power needed to get to required heading.
      */
     public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
         targetHeading = desiredHeading;  // Save for telemetry

         // Determine the heading current error
//         headingError = targetHeading - getHeading();

         // Normalize the error to be within +/- 180 degrees
         while (headingError > 180)  headingError -= 360;
         while (headingError <= -180) headingError += 360;

         // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
         return Range.clip(headingError * proportionalGain, -1, 1);
     }

//     /**
//      * Take separate drive (fwd/rev) and turn (right/left) requests,
//      * combines them, and applies the appropriate speed commands to the left and right wheel motors.
//      * @param drive forward motor speed
//      * @param turn  clockwise turning motor speed.
//      */
//     public double getHeading() {
//         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//         return orientation.getYaw(AngleUnit.DEGREES);
//     }
     public void moveRobot(double drive, double turn) {
         driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
         turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

         leftSpeed  = drive - turn;
         rightSpeed = drive + turn;

         // Scale speeds down if either one exceeds +/- 1.0;
         double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
         if (max > 1.0)
         {
             leftSpeed /= max;
             rightSpeed /= max;
         }

         leftFront.setPower(leftSpeed);
         leftBack.setPower(leftSpeed);
         rightFront.setPower(rightSpeed);
         rightBack.setPower(rightSpeed);
     }
     public void initGyro(){
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
         parameters.loggingEnabled      = true;
         parameters.loggingTag          = "IMU";
         parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
         //
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);
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
     public void liftMotor_Claw_Reset () {
         claw.setPosition(0); // leave claw open
         liftMotor.setTargetPosition(0); // bring slide down
     }
     public void liftMotor_basketServo_Reset () {
         basketServo.setPosition(0); // return basket
         liftMotor.setTargetPosition(0); // bring slide down
     }
     public void basket_place_sequence () {
         extensionMotor.setTargetPosition(1); // find the set position for the 1 inch adjustment
         basketServo.setPosition(1); // dump basket
         sleep(500);
         basketServo.setPosition(0); // pull back basket
         extensionMotor.setTargetPosition(0); // return extension motor
     }
     public void intake () {
         intake.setPower(-.5);
     }
     public void outtake () {
         intake.setPower(.5);
     }
     public void stopIntakeOROuttake () {
         intake.setPower(0);
     }
 }