package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 *  Mecanum drive methods and constants used to operate the drivetrain in autonomous mode.  Intended
 *  to encapsulate common code in one place, to be invoked from several autonomous programs.  Not an opMode.
 */
public class MecanumChassis {

    // Motor encoder, gearbox, drive gear and wheel physical constants
    static final double COUNTS_PER_MOTOR_REV = 28.0; // Encoder countable events / rev
    static final double GEARBOX_REDUCTION = 13.7; // goBILDA 5202-0002-0014
    static final double FINAL_DRIVE_REDUCTION = 2.0;  // Final drive - bevel gears
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // GoBilda Mecanum wheels

    // Chassis physical constants for rotation
    static final double ROTATION_DIAMETER = 19.25; // Diagonal through center of rotation


    // Empirically determined slippage adjustments
    static final double ADJUST_FWD_BACK = 1.0;  // Adjustment for forward - backward motion
    static final double ADJUST_STRAFE = 1.05; // Adjustment for strafing (side-to-side)
    static final double ADJUST_ROTATE = 1.43; // Adjustment for rotation (on axis)

    // The derived final multipliers
    static final double COUNTS_PER_INCH_F_B = ((COUNTS_PER_MOTOR_REV * GEARBOX_REDUCTION * FINAL_DRIVE_REDUCTION) /
            (Math.PI * WHEEL_DIAMETER_INCHES)) * ADJUST_FWD_BACK;
    static final double COUNTS_PER_INCH_STRAFE = ((COUNTS_PER_MOTOR_REV * GEARBOX_REDUCTION * FINAL_DRIVE_REDUCTION) /
            (Math.PI * WHEEL_DIAMETER_INCHES)) * ADJUST_STRAFE;
    static final double COUNTS_PER_DEGREE = ((COUNTS_PER_INCH_F_B * Math.PI * ROTATION_DIAMETER) / 360.0)
            * ADJUST_ROTATE;

    static final double K_PROP_R = (1.0 / 180.0); // this constant is for the gyro rotation
    static final double K_PROP_R_D = (1.0 / 25.0); // this constant is for the gyro rotation in gyro drive
    static final double K_PROP_R_S = (1.0 / 180.0); // this constant is for the gyro rotation in gyro strafe
    double K_PROP_F_B = 0.05; // this is a var that depends on the distance that the robot has to move
    double K_PROP_L_R = 0.05; // this is a var that depends on the distance that the robot has to move

    // variables for gyro movement
    static float gyroTarAngle;
    static int encoderDistance;
    static double tarAngleCurve;
    static double gyroSpeed;
    static boolean startRotate = false;
    static Telemetry telemetry;
    static Orientation angles;
    static Acceleration gravity;


    Direction gyroDirection;

    /*
     * Constants to use in calls to longDrive(), strafeDrive(), and rotate() methods.  Available to
     * callers as MecanumChassis.Direction.*
     */
    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    private DcMotor frontRightObj;
    private DcMotor frontLeftObj;
    private DcMotor rearLeftObj;
    private DcMotor rearRightObj;

    BNO055IMU imu;

    /**
     * Constructor for MecanumChassis.  Call with hardware map object, and four string arguments, containing the drivetrain
     * motor names from the Robot Controller configuration.
     *
     * @param hardwareMap hardwareMap object from calling autonomous program
     * @param frontLeft   Configuration name for front left drive motor
     * @param frontRight  Configuration name for front right drive motor
     * @param rearLeft    Configuration name for rear left drive motor
     * @param rearRight   Configuration name for rear right drive motor
     */
    MecanumChassis(HardwareMap hardwareMap, String frontLeft, String frontRight, String rearLeft, String rearRight) {

        //Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*  Instantiate motor objects, and initialize them in every way we can think of.
         */
        frontLeftObj = hardwareMap.get(DcMotor.class, frontLeft);
        frontRightObj = hardwareMap.get(DcMotor.class, frontRight);
        rearLeftObj = hardwareMap.get(DcMotor.class, rearLeft);
        rearRightObj = hardwareMap.get(DcMotor.class, rearRight);

        frontRightObj.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightObj.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftObj.setPower(0.0);
        frontRightObj.setPower(0.0);
        rearLeftObj.setPower(0.0);
        rearRightObj.setPower(0.0);

        frontLeftObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftObj.setTargetPosition(0);
        frontRightObj.setTargetPosition(0);
        rearLeftObj.setTargetPosition(0);
        rearRightObj.setTargetPosition(0);

        frontLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * stopDrive() - Sets motor power to zero immediately.  No changes to modes, or encoder counts.
     */
    public void stopDrive() {
        frontLeftObj.setPower(0.0);
        frontRightObj.setPower(0.0);
        rearLeftObj.setPower(0.0);
        rearRightObj.setPower(0.0);
    }

    /**
     * isBusy() - Returns TRUE if the robot is in motion (any wheels are turning under power)
     *
     * @return TRUE if robot is moving under power (i.e., any motor is active)
     */
    public boolean isBusy() {
        if (frontLeftObj.isBusy() || frontRightObj.isBusy() || rearLeftObj.isBusy() || rearRightObj.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * longDrive() - Move the robot forward or backward (i.e., parallel to the long axis)
     *
     * @param direction FORWARD or BACKWARD
     * @param distance  in inches
     * @param speed     zero to one
     */
    public void longDrive(Direction direction, double distance, double speed) {

        frontRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int frontLeftPos;
        int frontRightPos;
        int rearLeftPos;
        int rearRightPos;

        if (direction == Direction.BACKWARD) {
            distance = -distance;
        }
        frontLeftPos = frontLeftObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_F_B);
        frontRightPos = frontRightObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_F_B);
        rearLeftPos = rearLeftObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_F_B);
        rearRightPos = rearRightObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_F_B);

        frontLeftObj.setPower(speed);
        frontRightObj.setPower(speed);
        rearLeftObj.setPower(speed);
        rearRightObj.setPower(speed);

        frontLeftObj.setTargetPosition(frontLeftPos);
        frontRightObj.setTargetPosition(frontRightPos);
        rearLeftObj.setTargetPosition(rearLeftPos);
        rearRightObj.setTargetPosition(rearRightPos);
    }

    /**
     * strafeDrive() - Move the robot sideways, without rotation (i.e., perpendicular to the long axis)
     *
     * @param direction LEFT or RIGHT
     * @param distance  in inches
     * @param speed     zero to one
     */
    public void strafeDrive(Direction direction, double distance, double speed) {

        frontRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int frontLeftPos;
        int frontRightPos;
        int rearLeftPos;
        int rearRightPos;

        if (direction == Direction.LEFT) {
            distance = -distance;
        }
        frontLeftPos = frontLeftObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_STRAFE);
        frontRightPos = frontRightObj.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH_STRAFE);
        rearLeftPos = rearLeftObj.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH_STRAFE);
        rearRightPos = rearRightObj.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_STRAFE);

        frontLeftObj.setPower(speed);
        frontRightObj.setPower(speed);
        rearLeftObj.setPower(speed);
        rearRightObj.setPower(speed);

        frontLeftObj.setTargetPosition(frontLeftPos);
        frontRightObj.setTargetPosition(frontRightPos);
        rearLeftObj.setTargetPosition(rearLeftPos);
        rearRightObj.setTargetPosition(rearRightPos);
    }

    /**
     * rotate() - Rotate the robot around its center.
     *
     * @param direction CLOCKWISE or COUNTERCLOCKWISE
     * @param degrees   amount of rotation (positive number)
     * @param speed     zero to one
     */
    public void rotate(Direction direction, double degrees, double speed) {
        frontRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int frontLeftPos;
        int frontRightPos;
        int rearLeftPos;
        int rearRightPos;

        if (direction == Direction.COUNTERCLOCKWISE) {
            degrees = -degrees;
        }
        frontLeftPos = frontLeftObj.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        frontRightPos = frontRightObj.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
        rearLeftPos = rearLeftObj.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        rearRightPos = rearRightObj.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);

        frontLeftObj.setPower(speed);
        frontRightObj.setPower(speed);
        rearLeftObj.setPower(speed);
        rearRightObj.setPower(speed);

        frontLeftObj.setTargetPosition(frontLeftPos);
        frontRightObj.setTargetPosition(frontRightPos);
        rearLeftObj.setTargetPosition(rearLeftPos);
        rearRightObj.setTargetPosition(rearRightPos);
    }


    /**
     * getCurrentHeading() - gets the current heading of the robot
     *
     * @return the current heading
     */
    public float getCurrentHeading() {
        // gets the orientation and gets the heading value
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }

    /**
     * gyroRotate() - Rotate the robot around its center using the IMU gyro, relative to its
     * current orientation, by addedAngle degrees.
     *
     * @param addedAngle amount of rotation clockwise
     * @param speed      zero to one
     */
    public void gyroRotate(float addedAngle, double speed) {
        // stop the motors before running the motion
        frontLeftObj.setPower(0);
        frontRightObj.setPower(0);
        rearLeftObj.setPower(0);
        rearRightObj.setPower(0);

        // sets the motor mode to run without encoder
        frontRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set variables for the finishRotate()
        gyroSpeed = speed;
        gyroTarAngle = -addedAngle + getCurrentHeading();
        if(Math.abs(gyroTarAngle) > 180){
            gyroTarAngle = -Math.copySign(360.0f-Math.abs(gyroTarAngle),gyroTarAngle);
        }

    }

    /**
     * gyroRotateTo - rotates to a angle not relative to the current angle
     *
     * @param tarAngle
     * @param speed
     */
    public void gyroRotateTo(float tarAngle, double speed) {
        // stop the motors before running the motion
        frontLeftObj.setPower(0);
        frontRightObj.setPower(0);
        rearLeftObj.setPower(0);
        rearRightObj.setPower(0);

        // sets the motor mode to run without encoder
        frontRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set variables for the finishRotate()
        gyroSpeed = speed;
        gyroTarAngle = tarAngle;
    }
    /**
     * finishRotate() - used like the isBusy() function
     *
     * @return true if the motors are running or false if they are not
     */
    public boolean finishRotate() {
        // gets the error (how far off the angle is)
        /*
         * Magnitude of "error" will be the number of degrees of rotation required
         * to match the current heading to the target angle.
         *
         * The sign of "error" will determine the direction of rotation.  If more than 180 degrees
         * of rotation is required, reverse the direction of the rotation, and recalculate the magnitude.
         *
         * When error is positive, we turn clockwise.
         */
        float error = getCurrentHeading() - gyroTarAngle;

        // changes the error to match the shortest path to turn
        if (Math.abs(error) > 180) {
            error = -Math.copySign(360 - Math.abs(error), error);
        }

        //calculates the speed of the motors
//        float rotVal = error * (float) gyroSpeed * (float) K_PROP_R;

        //checks if it is done
        if (Math.abs(error) < 0.5) {

            // if it is done, stop the motors and return false
            frontLeftObj.setPower(0);
            frontRightObj.setPower(0);
            rearLeftObj.setPower(0);
            rearRightObj.setPower(0);

            return false;
        } else {
            //if it isn't done then change the power in the motors and return true
            frontRightObj.setPower(-Math.copySign(gyroSpeed, error));
            frontLeftObj.setPower(Math.copySign(gyroSpeed, error));
            rearRightObj.setPower(-Math.copySign(gyroSpeed, error));
            rearLeftObj.setPower(Math.copySign(gyroSpeed, error));

            return true;
        }
    }

    /**
     * gyroDrive() - Move forward or backward and stay straight using the IMU gyro
     *
     * @param tele used for debugging
     * @param direction the direction of the movement
     * @param distance  the amount of inches forward
     * @param speed     zero to one
     */
    public void gyroDrive(Telemetry tele, Direction direction, double distance, double speed) {
        telemetry = tele;

        // resets the encoder values
        frontLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the motor modes to run without encoder
        frontRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if the direction is backward then reverse the speed and distance signs
        if (distance < 0) {
            speed = -speed;             // multiplying the speed by -1
        }
        if (direction == Direction.BACKWARD) {
            speed = -speed;             // multiplying the speed by -1
        }

        //set variables for finishDrive()
        startRotate = false;                                    //
        encoderDistance = (int) (distance * COUNTS_PER_INCH_F_B);
        gyroTarAngle = getCurrentHeading();
        gyroSpeed = speed;

        // set power to go forward
        frontLeftObj.setPower(speed);
        frontRightObj.setPower(speed);
        rearLeftObj.setPower(speed);
        rearRightObj.setPower(speed);
    }

    /**
     * finishDrive() - move forward/backwards and straighten it out using our gyro
     *
     * @param telemetry used for debugging
     * @return true if the motors are running or false if they are not
     */
    public boolean finishDrive(Telemetry telemetry) {
        // find the positions of each motor so we can test if the forward motion is done
        int frontRightPos = frontRightObj.getCurrentPosition();
        int frontLeftPos = frontRightObj.getCurrentPosition();
        int rearRightPos = rearLeftObj.getCurrentPosition();
        int rearLeftPos = rearLeftObj.getCurrentPosition();

        // find the error of the angle and find the average of all the encoder valuues
        double currentDistance = (Math.abs(frontLeftPos) + Math.abs(frontRightPos) + Math.abs(rearLeftPos) + Math.abs(rearRightPos))/4;
        float angleError = getCurrentHeading() - gyroTarAngle;

        // fix the angleError so it doesn't go past 180 degrees
        if (Math.abs(angleError) > 180.0) {
            angleError = -Math.copySign(360.0f - Math.abs(angleError), angleError);
        }

        // set the rotation variable so that we can adjust the speed of the motors
        double rotVal = angleError * K_PROP_R_D;
//        // add telemetry for debugging
//        telemetry.addData("Current Heading", getCurrentHeading());
//        telemetry.addData("Angle Error", angleError);
//        telemetry.addData("Rotation Variable", rotVal);
//        telemetry.update();

        // test if the forward motion is just finished by checking if the currentDistance is greater than encoderDistance, and if we haven't started the final rotation at the end.
        if (currentDistance >= Math.abs(encoderDistance) && !startRotate) {
            frontLeftObj.setPower(0);
            frontRightObj.setPower(0);
            rearLeftObj.setPower(0);
            rearRightObj.setPower(0);
            // this code is for the rotation at the end of the movement
            // if it is start the rotation
            gyroRotateTo(gyroTarAngle, gyroSpeed/5);
            startRotate = true;
            return(true);
        } else if (startRotate) {
            // check if the rotation has ended
            boolean endReturn = finishRotate();
//            telemetry.addData("Return value of finishRotate()", endReturn);
//            telemetry.update();
            return(endReturn);
        } else {
            double motorSpeed = gyroSpeed;
            // if the current distance is less than 5% or is greater than 95% then we want to reduce the speed
            if (currentDistance <= Math.abs(encoderDistance/20) || currentDistance >= Math.abs(encoderDistance*19/20)) {
                motorSpeed /= 2;
            }
            // set power to go forward and to align the robot so it stays safe
            frontLeftObj.setPower(motorSpeed + rotVal);
            frontRightObj.setPower(motorSpeed - rotVal);
            rearLeftObj.setPower(motorSpeed + rotVal);
            rearRightObj.setPower(motorSpeed - rotVal);
            return true;
        }
    }

    /**
     * gyroStrafe() - Move left or right and stay straight using the IMU gyro
     *
     * @param tele used for debugging
     * @param direction the direction of the movement either forward or backward
     * @param distance  the amount of inches in the direction given
     * @param speed     zero to one
     */
    public void gyroStrafe(Telemetry tele, Direction direction, double distance, double speed) {
        telemetry = tele;

        frontLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the motor modes to run without encoder
        frontRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //if the distance is negative than the speed should be going backwards
        if (distance < 0) {
            speed = -speed;
        }
        //if the directions is left than the speed should be going backwards
        if (direction == Direction.LEFT) {
            speed = -speed;
        }

        //set variables for finishDrive()
        startRotate = false;
        encoderDistance = (int) (distance * COUNTS_PER_INCH_F_B);
        gyroTarAngle = getCurrentHeading();
        gyroSpeed = speed;

        // set power to go right if speed is positive
        frontLeftObj.setPower(speed);
        frontRightObj.setPower(-speed);
        rearLeftObj.setPower(-speed);
        rearRightObj.setPower(speed);
    }

    /**
     * finishStrafe() - move left/right and straighten it out using our gyro
     *
     * @return true if the motors are running or false if they are not
     */
    public boolean finishStrafe(Telemetry telemetry) {
        // the positions of each motor
        int frontLeftPos = frontLeftObj.getCurrentPosition();
        int frontRightPos = frontRightObj.getCurrentPosition();
        int rearLeftPos = rearLeftObj.getCurrentPosition();
        int rearRightPos = rearRightObj.getCurrentPosition();

        double currentDistance = (Math.abs(frontLeftPos) + Math.abs(frontRightPos) + Math.abs(rearLeftPos) + Math.abs(rearRightPos))/4;
        // define variable that help to find the driving variables
        float angleError = getCurrentHeading() - gyroTarAngle;

        // fix the angleError
        if (Math.abs(angleError) > 180.0) {
            angleError = -Math.copySign(360.0f - Math.abs(angleError), angleError);
        }

        // set the rotation variable used
        double rotVal = angleError * K_PROP_R_D;

//        telemetry.addData("Current Heading", getCurrentHeading());
//        telemetry.addData("Angle Error", angleError);
//        telemetry.addData("Rotation Variable", rotVal);
//        telemetry.update();

        // test if the sideways motion is done by checking the front two wheels
        if (currentDistance >= Math.abs(encoderDistance) && !startRotate) {
            frontLeftObj.setPower(0);
            frontRightObj.setPower(0);
            rearLeftObj.setPower(0);
            rearRightObj.setPower(0);

            // if it is start the rotation
            gyroRotateTo(gyroTarAngle, gyroSpeed/5);
            startRotate = true;
            return (true);
        } else if (startRotate) {
            // check if the rotation has ended
            boolean endReturn = finishRotate();
            return(endReturn);
        } else {
            double motorSpeed = gyroSpeed;
            if (currentDistance <= Math.abs(encoderDistance/20) || currentDistance >= Math.abs(encoderDistance*19/20)) {
                motorSpeed /= 2;
            }
            // set power to go forward
            frontLeftObj.setPower(motorSpeed + rotVal);
            frontRightObj.setPower(-motorSpeed - rotVal);
            rearLeftObj.setPower(-motorSpeed + rotVal);
            rearRightObj.setPower(motorSpeed - rotVal);

            return true;
        }
    }

    /**
     * curve() - a method that we use in our super auton to strafe and rotate at the same time.
     *
     * @param tele         telemetry for debugging
     * @param direction    the direction of the rotation in the curve
     * @param degrees      the amount of degrees we are going to curve
     * @param speed        the speed of the strafe that we will do while rotating
     * @param mod          this variable is how fast we rotate, this will be subtracted or added to speed for rotation
     */
    public void curve(Telemetry tele, Direction direction, double degrees, double speed, double mod) {

        telemetry = tele;

        // this turns all the modes of the motors to RUN_USING_ENCODER
        frontRightObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // if the direction that was given is COUNTERCLOCKWISE we want to reverse the amount of degrees and the "modulator"
        if (direction == Direction.COUNTERCLOCKWISE) {
            degrees = -degrees;
            mod = -mod;
        }

        // set the target angle inside tarAngleCurve (for curveBusy())
        tarAngleCurve = getCurrentHeading() + degrees;

        // telemetry that is for debugging
        telemetry.addData("tarAngleCurve value", tarAngleCurve);
        telemetry.update();

        // set the powers for all the motors
        frontLeftObj.setPower(speed + mod);
        frontRightObj.setPower(-speed - mod);
        rearLeftObj.setPower(-speed + mod);
        rearRightObj.setPower(speed - mod);

    }

    /**
     * curveBusy() - tells if the function curve is still executing.
     *
     * @param tele telemetry for debugging
     * @return true if curve is still executing and false if it isn't
     */
    public boolean curveBusy(Telemetry tele){

        telemetry = tele;

        // calculate the error
        float error = (float)tarAngleCurve - getCurrentHeading();
        if (Math.abs(error) > 180) {
            error = -Math.copySign(360 - Math.abs(error), error);
        }

        // telemetry for debugging
        telemetry.addData("error", error);
        telemetry.addData("heading", getCurrentHeading());
        telemetry.update();

        /* if the error is greater than 2 or less than -2 then we want to say that it is still moving and it is busy.
        *  else we want to stop all the motors and say that the motors aren't busy
        */
        if(Math.abs(error) > 2){
            return true;
        }else{
            frontLeftObj.setPower(0);
            frontRightObj.setPower(0);
            rearLeftObj.setPower(0);
            rearRightObj.setPower(0);
            return false;
        }
    }
    //unused method that we were going to use to compose gyro telemetry
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}