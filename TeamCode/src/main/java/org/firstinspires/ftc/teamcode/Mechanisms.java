package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *  Mechanisms methods and constants used to operate the mechanisms in autonomous mode.  Intended
 *  to encapsulate common code in one place, to be invoked from several autonomous programs.  Not an opMode.
 */
public class Mechanisms {
    //initing vars for servo
    private Servo FrontHook;
    private Servo BackHook;
    private Servo LeftIntakeServo;
    private Servo RightIntakeServo;

    // Motor encoder, gearbox, drive gear and wheel physical constants


    //A lower number means a higher position for the hooks
    private double FRONT_HOOK_PARK          = 0.17  ;        // Park location (full up) for servo hook
    private double FRONT_HOOK_DOWN          = 0.65  ;        // Foundation drag position for servo hook
    private double FRONT_HOOK_STONE         = 0.57  ;        // Stone drag position for servo hook

    //A higher number means a higher position for the hooks
    private double BACK_HOOK_PARK           = 0.91  ;        // Park location (full up) for servo hook
    private double BACK_HOOK_DOWN           = 0.35  ;        // Foundation drag position for servo hook
    private double BACK_HOOK_STONE          = 0.45  ;        // Stone drag position for servo hook

    //A higher number means closer to the ground near stone position
    private double RIGHT_INTAKE_SERVO_PARK  = 0.16  ;        // Park location (full up) for the intake servo
    private double RIGHT_INTAKE_SERVO_MID1  = 0.45  ;        // Middle position for a action yet to be determined.
    private double RIGHT_INTAKE_SERVO_MID2  = 0.35  ;        // Middle position for a action yet to be determined.
    private double RIGHT_INTAKE_SERVO_STONE = 0.47  ;        // Stone pickup location for the intake servo

    //A lower number means closer to the ground near stone position
    private double LEFT_INTAKE_SERVO_PARK   = 0.87  ;        // Park location (full up) for the intake servo
    private double LEFT_INTAKE_SERVO_MID1   = 0.70  ;        // Middle position for a action yet to be determined.
    private double LEFT_INTAKE_SERVO_MID2   = 0.60  ;        // Middle position for a action yet to be determined.
    private double LEFT_INTAKE_SERVO_STONE  = 0.57 ;        // Stone pickup location for the intake servo

    private int isBusyTime = 0;


    /**
     * Constructor for Mechanisms.  Call with hardware map object, and two string arguments, containing the servo
     * names from the Robot Controller configuration.
     *
     * @param hardwareMap hardwareMap object from calling autonomous program
     * @param frontServo Configuration name for front hook
     * @param backServo Configuration name for back hook
     */
    Mechanisms(HardwareMap hardwareMap, String frontServo, String backServo, String rIntakeS, String lIntakeS){
/*  Instantiate servo objects, and initialize them in every way we can think of.
 */
        FrontHook  = hardwareMap.get(Servo.class, frontServo);
        BackHook = hardwareMap.get(Servo.class, backServo);
        LeftIntakeServo =  hardwareMap.get(Servo.class, lIntakeS);
        RightIntakeServo = hardwareMap.get(Servo.class, rIntakeS);

        //Sets hooks and intake servos to their respective park positions
        FrontHook.setPosition(FRONT_HOOK_PARK);
        BackHook.setPosition(BACK_HOOK_PARK);
        LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_PARK);
        RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_PARK);
    }

    /**
     * HookUp() - Brings Hooks Up.
     */
    public void hookUp(){
        FrontHook.setPosition(FRONT_HOOK_PARK);
        BackHook.setPosition(BACK_HOOK_PARK);
        //isBusyTimer = time + 1000;
    }
    /**
     * isBusy() - Brings Hooks Up.
     */
    public boolean isBusy(){
        //return(FrontHook.getPosition() == frontTargetPos || BackHook.getPosition() == backTargetPos);
        return(false);
        //return(isBusyTimer<time);
    }
    /**
     * HookDownStone() - Sets the hooks to grab stones.
     */
    public void hookDownStone(){
        FrontHook.setPosition(FRONT_HOOK_STONE);
        BackHook.setPosition(BACK_HOOK_STONE);
        //isBusyTimer = time + 1000;
    }

    /**
     * HookDownFoundation - Sets the hooks to grab the foundation.
     */
    public void hookDownFoundation(){
        FrontHook.setPosition(FRONT_HOOK_DOWN);
        BackHook.setPosition(BACK_HOOK_DOWN);
        //isBusyTimer = time + 1000;
    }

    /**
     * intakeDown - Brings the intake servos down
     */
    public void intakeDown(){
        LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_STONE);
        RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_STONE);
        //isBusyTimer = time + 1000;
    }

    /**
     * intakeUp - Brings the intake servos up
     */
    public void intakeUp(){
        LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_PARK);
        RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_PARK);
        //isBusyTimer = time + 1000;
    }
}
