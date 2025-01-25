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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
@Autonomous(name = "AutoITD_Contingency")
public class AutoITD_Contingency extends LinearOpMode {

    public SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
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
        RED_BLUE_RIGHTSTRAFE,
        RED_BLUE_LEFT,
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(true);

        // Define and Initialize Motors/Sensors/Servos
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        PivotServo = hardwareMap.get(CRServo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        while (opModeIsActive() && runonce == false) {
            if ((startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) || (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) || (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHTSTRAFE)) {
                telemetry.addData("Running >> ", startPosition.getClass());
                telemetry.update();
                if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) {
                    // NOTE: You may have to check the startpose to see if this strafing will work
                    // Facing forwards
                    robot.strafe(25,0.5,0.5);
                    robot.drive(52,0.4,0.5);
                    robot.turnTo(90,0.5,0.5);
                    basketServo.setPosition(1);
                    robot.drive(-12,0.5,0.5);
                    robot.drive(-6.3,0.3,0.5);
                    sleep(500);
                    runonce = true;
                }
                if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHTSTRAFE) {
                    // Facing forwards
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
                if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) {
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
                    robot.drive(20, 0.5, 0.25);
                    robot.turnTo(180,0.5,0.5);
                    // ======================= START LIMELIGHT
                    /* Starts polling for data.  If you neglect to call start(), getLatestResult() will return null. */
                    robot.strafe(-45, 0.55, 0.25);
                    sleep(500);
                    runonce = true;
                }
            }
        }
    }

    public void selectStartingPosition () {
        boolean selected = false;
        while (!isStopRequested() || (selected)) {
            telemetry.addLine("State Tournament - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Red/Blue Right(StrafeOnly)", "(^)");
//            telemetry.addData("    Square Testing ", "(v)");
            telemetry.addData("    Red/Blue Left    ", "(<)");
            telemetry.addData("    Red/Blue Right  ", "(>)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHTSTRAFE;
                telemetry.addData("Red/Blue Right(StrafeOnly): ", "selected");
                telemetry.update();
                selected = true;
                break;
            }
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
        while (liftMotor.getCurrentPosition() < 3400){
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
        basketServo.setPosition(0.34); // return basket
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
        while ((PivotServo.getPower() == -0.8) && (runonce == false)) {
            sleep(1000); // wait for extension motor
            // NOTE: this might affect the previous while loop - which is why I implemented the runonce
            intake.setPower(-0.7);
            sleep(500);
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