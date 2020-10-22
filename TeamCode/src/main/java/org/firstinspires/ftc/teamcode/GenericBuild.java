package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 *  GenericBuild methods encapsulate drive pattern for Building Zone autonomous in one place, to be used
 *  with different distances (magnitude and direction) for either alliance color.  Not an opMode.
 */
public class GenericBuild {

    /*
     * Constants to use in calls to constructor.  Available to
     * callers as GenericBuild.Alliance.*
     */
    public enum Alliance {
        BLUE,
        RED
    }

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // this timer is for servos because they don't have a way to check if they are still moving
    private Telemetry telemetry;                                                      // variable for telemetry
    private int allianceIndex;                                                        // this tells us if it is blue or red side
    private HardwareMap hwmap;                                                        // this is our hardware map variable

    private Mechanisms mechanisms;
    private MecanumChassis drivetrain;
    //init drivetrain and mechanisms

    private double startingHeading;
    // SPEED is the base speed of the auton, in our code we might divide speed by a certain number to insure stability
    private static double SPEED = 0.45;

    double[][] movement = new double[][]{ // movement of robot for red and blue sides
            /* Blue  */ {18 /*left*/, -24.5 /*forward*/, 12 /*left*/, 5 /*left (slow)*/, 3  /*right (slow)*/, 0, 38 /*right*/, 0, 0 /*CA-IA Clockwise*/, -35.0 /*backward*/, 47.5 /*left*/, -26.25 /*forward*/, 14 /*right*/, -21.5 /*backward*/, 32 /*right*/, -25 /*backward*/},
            /* Red   */ {18 /*left*/, 24.5  /*forward*/, 12 /*left*/, 5 /*left (slow)*/, 3  /*right (slow)*/, 0, 38 /*right*/, 0, 0 /*CA-IA Clockwise*/, 35.0  /*backward*/, 47.5 /*left*/, 26.25  /*forward*/, 14 /*right*/, 21.5  /*backward*/, 32 /*right*/, 25  /*backward*/}
    };

    /**
     * Constructor for GenericBuild.  Call with hardware map object, and the alliance color.
     *
     * @param hardwareMap hardwareMap object from calling autonomous program
     * @param allianceColor blue or red, from Alliance enumeration
     */
    GenericBuild(HardwareMap hardwareMap, Telemetry tele, Alliance allianceColor ){
        hwmap = hardwareMap;
        telemetry = tele;


        /*
         * Signed drive distances are stored in a two-dimensional array, or "array of arrays".  Each
         * distance array will have as many distances as there are states in the drive pattern state
         * machine.
         *
         *   Alliance | Index
         *   ---------+-------
         *   Blue     |   0
         *   Red      |   1
         *
         * We set allianceIndex to 0 for Blue, or 1 for Red, so the array indexes work out:
         *      distance = distanceArray[allianceIndex][state]
         */
        if(allianceColor == Alliance.BLUE)
            allianceIndex = 0;
        else
            allianceIndex = 1;              // allianceColor is RED

    /*
     * Instantiate a drivetrain
     */
        drivetrain = new MecanumChassis(hwmap, "lFront",
                "rFront", "lBack", "rBack");
        mechanisms = new Mechanisms(hardwareMap, "fHook", "bHook", "rIntakeS", "lIntakeS");

    }

    /**
     * getEndState() - Provides caller with the state number that indicates motion is complete.
     *
     * @return the total number of states
     */
    public int getEndState(){
        return (movement[allianceIndex].length);
    }
    /**
     * cycle() - runs state machine movement code
     *
     * @param state the current state
     * @return new state
     */
    public int cycle(int state) {

        double speed = SPEED;
        int newState = state;
        switch (state) {
            case 0:
                //go toward the foundation
                startingHeading = drivetrain.getCurrentHeading();
                drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, movement[allianceIndex][state], speed);
                newState = 1;
                break;

            case 1:
                if (!drivetrain.isBusy()) {
                    // aligned to the place that we want to pull
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.FORWARD, movement[allianceIndex][state], speed); // go to skystone
                    newState = 2;
                }
                break;
            case 2:
                if (!drivetrain.finishDrive(telemetry)) {
                    // go to the foundation fast
                    drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, movement[allianceIndex][state], speed); // go to skystone
                    newState = 3;
                }
                break;
            case 3:
                if (!drivetrain.isBusy()) {
                    // move slowly toward the foundation to ensure correct positioning relative to the foundation
                    drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, movement[allianceIndex][state], speed/3);
                    newState = 4;
                }
                break;
            case 4:
                if (!drivetrain.isBusy()) {
                    // strafe right to be in the right position to grab the foundation
                    drivetrain.strafeDrive(MecanumChassis.Direction.RIGHT, movement[allianceIndex][state], speed/3);
                    newState = 5;
                }
                break;
            case 5:
                if (!drivetrain.isBusy()) {
                    // hook the foundation
                    mechanisms.hookDownFoundation();
                    timer.reset();
                    newState = 6;
                }
                break;
            case 6:
                if (timer.milliseconds() > 1000) {
                    // pull the foundation while strafing
                    drivetrain.strafeDrive(MecanumChassis.Direction.RIGHT, movement[allianceIndex][state], speed);
                    newState = 7;
                }
                break;
            case 7:
                if (!drivetrain.isBusy()) {
                    // raise the hooks
                    mechanisms.hookUp();
                    timer.reset();
                    newState = 8;
                }
                break;
            case 8:
                if (timer.milliseconds() > 1000) {
                    // go into the stones to grab the skystone
                    drivetrain.rotate(MecanumChassis.Direction.LEFT, drivetrain.getCurrentHeading() - startingHeading, speed);
                    newState = 9;
                }
                break;
            case 9:
                if (!drivetrain.isBusy()) {
                    // grab skystone and set timer for next movement
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.BACKWARD, movement[allianceIndex][state], speed);
                    newState = 10;
                }
                break;
            case 10:
                if(!drivetrain.finishDrive(telemetry)){
                    drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, movement[allianceIndex][state], speed);
                    newState = 11;
                }
                break;
            case 11:
                if(!drivetrain.isBusy()){
                    drivetrain.longDrive(MecanumChassis.Direction.FORWARD, movement[allianceIndex][state], speed);
                    newState = 12;
                }
                break;
            case 12:
                if(!drivetrain.isBusy()){
                    drivetrain.strafeDrive(MecanumChassis.Direction.RIGHT, movement[allianceIndex][state], speed);
                    newState = 13;
                }
                break;
            case 13:
                if(!drivetrain.isBusy()) {
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.BACKWARD, movement[allianceIndex][state], speed);
                    newState = 14;
                }
                break;
            case 14:
                if(!drivetrain.finishDrive(telemetry)){
                    drivetrain.strafeDrive(MecanumChassis.Direction.RIGHT, movement[allianceIndex][state], speed);
                    newState = 15;
                }
                break;
            case 15:
                if(!drivetrain.isBusy()){
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.BACKWARD, movement[allianceIndex][state], speed);
                    newState = 16;
                }
                break;
            case 16:
                if(!drivetrain.finishDrive(telemetry)){
                    newState = 17;
                }
                break;
            case 17:
            default:

        }
        return (newState);
    }

}
