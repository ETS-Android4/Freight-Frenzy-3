package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// import org.firstinspires.ftc.teamcode.Level;

@TeleOp(name="Main Control", group="Pushbot")

public class MainTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    BotConfig robot = new BotConfig();
    final double TOLERANCE = 0.1;

    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        // initialize hardware variables
        robot.init(hardwareMap);

        double straif;
        double forward;
        double turnLeft;
        double turnRight;
        LiftLevel lift = LiftLevel.PICKUP;


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //reset encoder
        robot.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Winch motor ",  "Starting at %7d",
                robot.winchMotor.getCurrentPosition());
        telemetry.update();


        robot.basket.setPosition(0.81);
        boolean blockHeld = false;
        boolean OVERRIDE = true;
        final LiftLevel[] liftArr = new LiftLevel[1];
        liftArr[0] = lift;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            straif = -gamepad1.left_stick_x;
            forward = -gamepad1.left_stick_y;
            turnLeft = gamepad1.left_trigger;
            turnRight = gamepad1.right_trigger;
            double turn = -turnLeft + turnRight;
            double angle_rad = angles.firstAngle * -Math.PI/180.0;
            boolean liftMoving = false;
            boolean isDetected = RobotUtils.checkCSensor(robot.cSensor);
            liftMoving = robot.winchMotor.isBusy();
//            RobotUtils ru = new RobotUtils();

            // transposes coordinates based on gyro for FIELD ORIENTED DRIVING
            double temp = straif;
            straif = straif*Math.cos(angle_rad) - forward*Math.sin(angle_rad);
            forward = temp*Math.sin(angle_rad) + forward*Math.cos(angle_rad);

            // tolerance
            if (Math.abs(straif) <= TOLERANCE) {
                straif = 0;
            }
            if (Math.abs(forward) <= TOLERANCE) {
                forward = 0;
            }

            double m1D = (forward-straif+turn)/3.0;
            double m2D = (forward+straif+turn)/3.0;
            double m3D = (forward+straif-turn)/3.0;
            double m4D = (forward-straif-turn)/3.0;


            // set power for wheels
            robot.leftFront.setPower(m1D);
            robot.rightBack.setPower(m4D);
            robot.leftBack.setPower(m2D);
            robot.rightFront.setPower(m3D);
            robot.winchMotor.setPower(0.4);

            //check and enable LED if something is detected in light sensor
            telemetry.addData("Sensor: ", RobotUtils.showNormalizedRGBA(robot.cSensor.getNormalizedColors()));
            telemetry.update();

            lift = liftArr[0];

            if(isDetected) {
                telemetry.addData("Is detected?", "true");
                telemetry.update();
                if (lift.equals(LiftLevel.PICKUP)) {
                    RobotUtils.moveLiftUp(lift, robot.basket, robot.winchMotor);
                    lift = LiftLevel.upLevel(lift);
                    liftArr[0] = lift;
                }
                blockHeld = true;
            }
//            if(blockHeld) {
//                robot.abductor.setPower(1);
//                //TODO: LED LIGHTS ACTIVATE HERE
//            }
            //move lift up
            if (gamepad1.a && !liftMoving) {

                RobotUtils.moveLiftUp(liftArr[0], robot.basket, robot.winchMotor);
                lift = LiftLevel.upLevel(lift);
                liftArr[0] = lift;

                telemetry.addData("lift level is "+ lift.toString(), "");
                telemetry.addData("target pos is "+ robot.winchMotor.getTargetPosition(), "");
                telemetry.update();

            }
            //move lift down
            else if (gamepad1.b && !liftMoving) {

                RobotUtils.moveLiftDown(liftArr[0], robot.basket, robot.winchMotor);
                lift = LiftLevel.downLevel(lift);
                liftArr[0] = lift;

                telemetry.addData("lift level is "+ lift.toString(), "");
                telemetry.addData("target pos is "+ robot.winchMotor.getTargetPosition(), "");
                telemetry.update();

            }
            //move abductor
            else if (gamepad1.right_bumper) {
                robot.abductor.setPower(1);
            }
            else if (gamepad1.left_bumper) {
                robot.abductor.setPower(-1);
            }
            //turbo mode
            else if (gamepad1.y && lift == LiftLevel.CARRY) {
                robot.leftFront.setPower (1);
                robot.rightBack.setPower (1);
                robot.leftBack.setPower  (1);
                robot.rightFront.setPower(1);
            }
            else if (gamepad1.x && lift == LiftLevel.CARRY) {
                robot.leftFront.setPower (-1);
                robot.rightBack.setPower (-1);
                robot.leftBack.setPower  (-1);
                robot.rightFront.setPower(-1);
            }
            //drop
            else if (gamepad1.x && (lift == LiftLevel.DROP_1 || lift==LiftLevel.DROP_3)) {
                robot.basket.setPosition(0);
                final LiftLevel tempLift = lift;
                new Thread( () -> {
                    sleep(1000);
                    if (tempLift.equals(LiftLevel.DROP_3) || tempLift.equals(LiftLevel.DROP_1)) {
                        RobotUtils.moveLiftDown(tempLift, robot.basket, robot.winchMotor);
                        liftArr[0] = LiftLevel.downLevel(tempLift);
                    }
                }).start();
                blockHeld = false;
            }
//            else if (gamepad1.dpad_down) {
//                // robot.winchMotor.setPower(0.7);
//                RobotUtils.moveLiftUp(lift, robot.basket, robot.winchMotor);
//                lift = LiftLevel.DROP_1;
//                // robot.winchMotor.setPower(0.4);
//            }
//            else if (gamepad1.dpad_up) {
//                // robot.winchMotor.setPower(0.5);
//                if(lift == LiftLevel.DROP_3)
//                    RobotUtils.moveLiftDown(lift, robot.basket, robot.winchMotor);
//                else
//                    RobotUtils.moveLiftUp(LiftLevel.DROP_1, robot.basket, robot.winchMotor);
//                lift = LiftLevel.DROP_3;
//                // robot.winchMotor.setPower(0.4);
//            }

            //stop the abductor
            if(!gamepad1.right_bumper && !gamepad1.left_bumper) {
                robot.abductor.setPower(0);
            }

            //MANUAL OVERRIDES
            //MANUAL OVERRIDES
//            if(gamepad2.dpad_down && OVERRIDE) {
//                robot.winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.winchMotor.setPower(-0.3);
////                robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if(gamepad2.dpad_up && OVERRIDE) {
//                robot.winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.winchMotor.setPower(0.3);
//            }
//            else if(OVERRIDE && !gamepad2.dpad_down && !gamepad2.dpad_up) {
//                robot.winchMotor.setPower(0);
//            }
//            if(gamepad2.x) {
//                robot.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                OVERRIDE = false;
//            }
        }
    }
}










