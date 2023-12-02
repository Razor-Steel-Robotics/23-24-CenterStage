package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous_Test")
//@Disabled
public class AutonomousCodeTest extends LinearOpMode {

    public static int driveCount = 0;
    public static int driveLoopCount = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2; //Was 19.2
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Drive function with 3 parameters
    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions (We currently need the bot to run like a tank drive)
            rightTarget = ((rightFrontDrive.getCurrentPosition())) + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = ((leftFrontDrive.getCurrentPosition())) + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            leftFrontDrive.setTargetPosition(leftTarget);
            //leftBackDrive.setTargetPosition(leftTarget);
            rightFrontDrive.setTargetPosition(rightTarget);
            //rightBackDrive.setTargetPosition(rightTarget);

            //switch to run to position mode

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //run to position at the designated power
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            //leftBackDrive.setPower(power);
            //rightBackDrive.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            }

            // set motor power back to 0
            //leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            //rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            driveLoopCount++;

        }
        driveCount++;
        // Format (%d (int), %s (string), %f (float)) allows for injecting variables into a string
        telemetry.addData("Successful Drive Count / Stuck in Drive Loop", "%d, %d", driveCount, driveLoopCount);
        telemetry.update();
    }

//Run Op Mode is the Main Function of Robots
    @Override
    public void runOpMode() {


        driveCount = 0;
        driveLoopCount = 0;
        //rightBackDrive = hardwareMap.get(DcMotor.class, "RR");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        //leftBackDrive = hardwareMap.get(DcMotor.class, "LR");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");

        telemetry.addData("Hi", "34");
        telemetry.update();

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            drive(1, 70,70);
            //segment 1
            drive(0.7, 30, 15);

            runtime.reset(); // reset elapsed time timer

            //segment 2 - lift arm, drive to shipping hub, outtake freight
            while (opModeIsActive() && runtime.seconds() <= 7) {

                /*
                //lift arm and hold
                Arm.setTargetPosition(120);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.3);
                */

                //drive forward for 1 second
                while (runtime.seconds() > 2 && runtime.seconds() <= 3) {
                    drive(0.4, 4, 4);
                }

                //run intake
                while (runtime.seconds() > 4 && runtime.seconds() <= 7) {
                    //Intake.setPower(-0.6);
                }

                // turn off arm and intake
                //Arm.setPower(0);
                //Intake.setPower(0);

                //segment 3 - reverse to get better angle
                //drive(0.7, -15, -30);

                //segment 4 - drive into warehouse
                //drive(1, 90, 90);
            }
        }
    }
}
