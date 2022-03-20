package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous

public class AutonomousBlue extends LinearOpMode {
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
        Object lock = new Object();
        double straif;
        double forward;
        double turnLeft;
        double turnRight;
        LiftLevel lift = LiftLevel.PICKUP;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";

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
        DcMotor lf = robot.leftFront;
        DcMotor lb = robot.leftBack;
        DcMotor rf = robot.rightFront;
        DcMotor rb = robot.rightBack;

        final double pwr = 0.3;
        final int rotTime = 1580;
        final int duckyTime = 3000;
        final int f1 = 1000;
        final int s1 = 3000;
        final int f2 = 1000;


        while (opModeIsActive()) {
            /*
            robot.winchMotor.setPower(0.4);
            */
            //go forward to drop thing
            RobotUtils.setMotors(lf,lb,rf,rb,-pwr,-pwr,-pwr,-pwr);
            // robot.basket.setPosition(0.55);
            sleep(f1);
            RobotUtils.stopMotors(lf,lb,rf,rb);

            //lift and drop

            final LiftLevel[] liftArr = new LiftLevel[1];
            liftArr[0] = lift;

            synchronized(lock) {
                robot.winchMotor.setPower(0.4);
                RobotUtils.moveLiftUp(liftArr[0], robot.basket, robot.winchMotor);
                lift = LiftLevel.upLevel(lift); // to carry
                liftArr[0] = lift;
                // sleep(100);
                RobotUtils.moveLiftUp(lift, robot.basket, robot.winchMotor);
                lift = LiftLevel.upLevel(lift); // to drop 3
                liftArr[0] = lift;
                sleep(3000);
                //drop object
                robot.basket.setPosition(0);
                final LiftLevel tempLift = lift;
                new Thread( () -> {
                    sleep(1000);
                    if (tempLift.equals(LiftLevel.DROP_3) || tempLift.equals(LiftLevel.DROP_1)) {
                        RobotUtils.moveLiftDown(tempLift, robot.basket, robot.winchMotor);
                        liftArr[0] = LiftLevel.downLevel(tempLift);
                    }
                }).start();
                sleep(1200);
                //END DROP
//                RobotUtils.moveLiftDown(lift, robot.basket, robot.winchMotor);
//                lift = LiftLevel.downLevel(lift);
//                sleep(2000);
//                RobotUtils.moveLiftDown(lift, robot.basket, robot.winchMotor);
//                lift = LiftLevel.downLevel(lift);
//                sleep(3000);
            }

            //go back to start position
            RobotUtils.setMotors(lf,lb,rf,rb,pwr,pwr,pwr,pwr);
            sleep(f1);
            RobotUtils.stopMotors(lf,lb,rf,rb);
            sleep(2500);
            RobotUtils.rotateT(robot, 90);
            sleep(1500);
            RobotUtils.setMotors(lf,lb,rf,rb,-pwr,-pwr,-pwr,-pwr);
            //go to duckies
            sleep(1200);
            RobotUtils.stopMotors(lf,lb,rf,rb);
            RobotUtils.rotateT(robot, 20);
            sleep(500);
            RobotUtils.setMotors(lf,lb,rf,rb,-pwr,-pwr,-pwr,-pwr);
            sleep(500);
            RobotUtils.stopMotors(lf,lb,rf,rb);
            return;
            //TODO: MAKE IT DROP TO A LOWER LEVEL WHEN DONE WITH AUTONOMOUS SO IT WORKS WITH MAINTELEOP
            /*
            //rotate 1pi radians
            RobotUtils.setMotors(lf,lb,rf,rb,pwr,pwr,-pwr,-pwr); //spin around 1pi radians
            sleep(rotTime);
            RobotUtils.stopMotors(lf,lb,rf,rb);
            sleep(100);

            //go to duckies
            RobotUtils.setMotors(lf,lb,rf,rb,-pwr,-pwr,-pwr,-pwr);
            sleep(s1);
            RobotUtils.stopMotors(lf,lb,rf, rb);
            //spin abductor
            robot.abductor.setPosition(1);
            sleep(duckyTime);
            robot.abductor.setPosition(0.5); //stop abductor
            RobotUtils.stopMotors(lf,lb,rf,rb);
            //go to storage thing
            RobotUtils.setMotors(lf,lb,rf,rb,pwr,-pwr,-pwr,pwr);
            sleep(f2);
            RobotUtils.stopMotors(lf,lb,rf,rb);
            return;
            */
        }

    }
}