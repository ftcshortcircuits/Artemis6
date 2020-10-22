package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 *  GenericLoad methods encapsulate drive pattern for Loading Zone autonomous in one place, to be used
 *  with different distances (magnitude and direction) for either alliance color.  Not an opMode.
 */
public class GenericLoad {

    /*
     * Constants to use in calls to constructor.  Available to
     * callers as GenericLoad.Alliance.*
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

    // SPEED is the base speed of the auton, in our code we might divide speed by a certain number to insure stability
    private static double SPEED = 0.45;

    double[][] movement = new double[][]{ // movement of robot for stone 1, stone 2 and stone 3
            /* Blue 1 */ {30.5   /*left*/, -8  /*forward*/, 3  /*left*/, 0, 13  /*right*/, 54  /*backward*/, 0, 78  /*forward*/, 15  /*left*/, 0, 15  /*right*/, 65  /*backward*/, 0, 7  /*left*/, 11  /*forward*/},
            /* Blue 2 */ {30.5   /*left*/, 0   /*forward*/, 3  /*left*/, 0, 13  /*right*/, 62  /*backward*/, 0, 86  /*forward*/, 15  /*left*/, 0, 15  /*right*/, 73  /*backward*/, 0, 7  /*left*/, 11  /*forward*/},
            /* Blue 3 */ {30.5   /*left*/, 8   /*forward*/, 3  /*left*/, 0, 13  /*right*/, 70  /*backward*/, 0, 86  /*forward*/, 15  /*left*/, 0, 15  /*right*/, 73  /*backward*/, 0, 7  /*left*/, 11  /*forward*/},
            /* Red  1 */ {30.5   /*left*/, 8   /*forward*/, 3  /*left*/, 0, 13  /*right*/, -54 /*backward*/, 0, -78 /*forward*/, 15  /*left*/, 0, 15  /*right*/, -65 /*backward*/, 0, 7  /*left*/, -11 /*forward*/},
            /* Red  2 */ {30.5   /*left*/, 0   /*forward*/, 3  /*left*/, 0, 13  /*right*/, -62 /*backward*/, 0, -86 /*forward*/, 15  /*left*/, 0, 15  /*right*/, -73 /*backward*/, 0, 7  /*left*/, -11 /*forward*/},
            /* Red  3 */ {30.5   /*left*/, -8  /*forward*/, 3  /*left*/, 0, 13  /*right*/, -70 /*backward*/, 0, -86 /*forward*/, 15  /*left*/, 0, 15  /*right*/, -73 /*backward*/, 0, 7  /*left*/, -11 /*forward*/}
    };

    /**
     * Constructor for GenericLoad.  Call with hardware map object, and the alliance color.
     *
     * @param hardwareMap hardwareMap object from calling autonomous program
     * @param allianceColor blue or red, from Alliance enumeration
     */
    GenericLoad(HardwareMap hardwareMap, Telemetry tele, Alliance allianceColor ){
        hwmap = hardwareMap;
        telemetry = tele;


        /*
         * Signed drive distances are stored in a two-dimensional array, or "array of arrays".  Each
         * distance array will have as many distances as there are states in the drive pattern state
         * machine.  There will be six distance arrays, one for each alliance and Skystone pattern:
         *
         *   Alliance | Stone | Index | Meaning
         *   ---------+-------+-------+--------------------
         *   Blue     |   1   |   0   | Skystones are 1 & 4
         *   Blue     |   2   |   1   | Skystones are 2 & 5
         *   Blue     |   3   |   2   | Skystones are 3 & 6
         *   Red      |   1   |   3   | Skystones are 1 & 4
         *   Red      |   2   |   4   | Skystones are 2 & 5
         *   Red      |   3   |   5   | Skystones are 3 & 6
         *
         * We set allianceIndex to -1 for Blue, or 2 for Red, so the array indexes work out:
         *      distance = distanceArray[allianceIndex+stone][state]
         * (NOTE: we don't get the Stone value in the constructor.)
         */
        if(allianceColor == Alliance.BLUE)
            allianceIndex = -1;
        else
            allianceIndex = 2;              // allianceColor is RED

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
     * @param stone the stone position
     * @return the total number of states
     */
    public int getEndState(int stone){
        return (movement[stone+allianceIndex].length);
    }
    /**
     * cycle() - runs state machine movement code
     *
     * @param state the current state
     * @return new state
     */
    public int cycle(int state, int stone) {

        double speed = SPEED;
        int newState = state;
        switch (state) {
            case 0:
                //go towards the stones
                drivetrain.gyroStrafe(telemetry, MecanumChassis.Direction.LEFT, movement[stone+allianceIndex][state], speed);
                newState = 1;
                break;

            case 1:
                if (!drivetrain.finishStrafe(telemetry)) {
                    // aligned to the skystone
                    drivetrain.longDrive(MecanumChassis.Direction.FORWARD, movement[stone+allianceIndex][state], speed); // go to skystone
                    newState = 2;
                }
                break;
            case 2:
                if (!drivetrain.isBusy()) {
                    // go into the stones to get ready to grab the skystone
                    drivetrain.gyroStrafe(telemetry, MecanumChassis.Direction.LEFT, movement[stone+allianceIndex][state], speed); // go to skystone
                    newState = 3;
                }
                break;
            case 3:
                if (!drivetrain.finishStrafe(telemetry)) {
                    // grab skystone and reset the timer so next movement is after the hook goes down
                    mechanisms.hookDownStone();
                    timer.reset();
                    newState = 4;
                }
                break;
            case 4:
                if (timer.milliseconds() >= 1000) {
                    // strafe right, away from the stones so you dont touch the skybridge
                    drivetrain.gyroStrafe(telemetry, MecanumChassis.Direction.RIGHT, movement[stone+allianceIndex][state], speed);
                    newState = 5;
                }
                break;
            case 5:
                if (!drivetrain.finishStrafe(telemetry)) {
                    // go backward and pass the skystone tape fully to score the skystone
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.BACKWARD, movement[stone+allianceIndex][state], speed);
                    newState = 6;
                }
                break;
            case 6:
                if (!drivetrain.finishDrive(telemetry)) {
                    // release the stone and set timer for next movement
                    mechanisms.hookUp();
                    timer.reset();
                    newState = 7;
                }
                break;
            //Second Set of Stones
            case 7:
                if (timer.milliseconds() >= 1000) {
                    // go forwards to second set of stones
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.FORWARD, movement[stone+allianceIndex][state], speed);
                    newState = 8;
                    //newState = 13; //Comment 7-11 back in when we are ready to collect the second stone.
                }
                break;
            case 8:
                if (!drivetrain.finishDrive(telemetry)) {
                    // go into the stones to grab the skystone
                    drivetrain.gyroStrafe(telemetry, MecanumChassis.Direction.LEFT, movement[stone+allianceIndex][state], speed);
                    newState = 9;
                }
                break;
            case 9:
                if (!drivetrain.finishStrafe(telemetry)) {
                    // grab skystone and set timer for next movement
                    mechanisms.hookDownStone();
                    timer.reset();
                    newState = 10;
                }
                break;
            case 10:
                if (timer.milliseconds() >= 1000) {
                    // strafe away from stones so you don't touch the skybridge
                    drivetrain.gyroStrafe(telemetry, MecanumChassis.Direction.RIGHT, movement[stone+allianceIndex][state], speed);
                    newState = 11;
                }
                break;
            case 11:
                if (!drivetrain.finishStrafe(telemetry)) {
                    // go past the skystone tape fully to score the second skystone
                    drivetrain.gyroDrive(telemetry, MecanumChassis.Direction.BACKWARD, movement[stone+allianceIndex][state], speed);
                    newState = 12;
                }
                break;
            case 12:
                if (!drivetrain.finishDrive(telemetry)) {
                    // release stone and set timer for next movement
                    mechanisms.hookUp();
                    mechanisms.intakeDown();
                    timer.reset();
                    newState = 13;
                }
                break;
            case 13:
                if (timer.milliseconds() >= 1000){
                    // park on tape
                    drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, movement[stone+allianceIndex][state], speed);
                    newState = 14;
                }
                break;
            case 14:
                if (!drivetrain.isBusy()){
                    // park on tape
                    drivetrain.longDrive(MecanumChassis.Direction.FORWARD, movement[stone+allianceIndex][state], speed);
                    newState = 15;
                }
                break;
            case 15:
                if (!drivetrain.isBusy()){
                    newState = 16;
                }
            default:

        }
        return (newState);
    }

}
