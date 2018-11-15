package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * By rachel on 1/19/18.
 * Based on code by team 4251
 */
public class GamepadEnhanced {

    public double AXIS_AS_BUTTON_THRESHHOLD = 0.75;


    public enum AXIS {
        AXIS_LEFT_STICK_X,
        AXIS_LEFT_STICK_Y,
        AXIS_RIGHT_STICK_X,
        AXIS_RIGHT_STICK_Y,
        AXIS_LEFT_TRIGGER,
        AXIS_RIGHT_TRIGGER
    }

    public enum BUTTON{
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        BUTTON_A,
        BUTTON_B,
        BUTTON_X,
        BUTTON_Y,
        BUTTON_GUIDE,
        BUTTON_START,
        BUTTON_BACK,
        BUTTON_LEFT_BUMPER,
        BUTTON_RIGHT_BUMPER,
        BUTTON_LEFT_STICK_BUTTON,
        BUTTON_RIGHT_STICK_BUTTON
    }

    Gamepad gamepad;

    // current states of buttons
    public float left_stick_x;
    public float left_stick_y;
    public float right_stick_x;
    public float right_stick_y;
    public boolean dpad_up;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;
    public boolean guide;
    public boolean start;
    public boolean back;
    public boolean left_bumper;
    public boolean right_bumper;
    public boolean left_stick_button;
    public boolean right_stick_button;
    public float left_trigger;
    public float right_trigger;

    // previous states of buttons
    public float prev_left_stick_x;
    public float prev_left_stick_y;
    public float prev_right_stick_x;
    public float prev_right_stick_y;
    public boolean prev_dpad_up;
    public boolean prev_dpad_down;
    public boolean prev_dpad_left;
    public boolean prev_dpad_right;
    public boolean prev_a;
    public boolean prev_b;
    public boolean prev_x;
    public boolean prev_y;
    public boolean prev_guide;
    public boolean prev_start;
    public boolean prev_back;
    public boolean prev_left_bumper;
    public boolean prev_right_bumper;
    public boolean prev_left_stick_button;
    public boolean prev_right_stick_button;
    public float prev_left_trigger;
    public float prev_right_trigger;


    /**
     * Default Constructor NOT TO BE USED
     */
    public GamepadEnhanced() {
        gamepad = null;
        System.out.println("No Gamepad passed to GamepadEnhanced class");

    }

    /**
     * The constructor that should be used.
     * @param gPad This Gamepad will be the controller that this instance of the class references.
     */
    public GamepadEnhanced(Gamepad gPad){
        gamepad = gPad;
        update(gPad);
    }

    /**
     * Used to change what gamepad this class references
     * @param gpad This Gamepad will be the controller that this instance of the class references.
     */
    public void setGamepad(Gamepad gpad){
        if(gpad == null){
            System.out.println("still no gamepad handed to GamepadEnhanced");
            return;
        }
        gamepad = gpad;
    }

    /**
     * MUST BE CALLED EVERY ITERATION OF DRIVER TO WORK!
     * This updates the states of all of the buttons, axes, and toggle functionality
     */
    public void update(Gamepad gamepad){
        if(gamepad == null) {
            System.out.println("still no gamepad handed to GamepadEnhanced");
            return;
        }

        // update all previous button states
        prev_left_stick_x = this.left_stick_x;
        prev_left_stick_y = this.left_stick_y;
        prev_right_stick_x = this.right_stick_x;
        prev_right_stick_y = this.right_stick_y;
        prev_dpad_up = this.dpad_up;
        prev_dpad_down = this.dpad_down;
        prev_dpad_left = this.dpad_left;
        prev_dpad_right = this.dpad_right;
        prev_a = this.a;
        prev_b = this.b;
        prev_x = this.x;
        prev_y = this.y;
        prev_guide = this.guide;
        prev_start = this.start;
        prev_back = this.back;
        prev_left_bumper = this.left_bumper;
        prev_right_bumper = this.right_bumper;
        prev_left_stick_button = this.left_stick_button;
        prev_right_stick_button = this.right_stick_button;
        prev_left_trigger = this.left_trigger;
        prev_right_trigger = this.right_trigger;

        // update all current button states
        this.left_stick_x = gamepad.left_stick_x;
        this.left_stick_y = gamepad.left_stick_y;
        this.right_stick_x = gamepad.right_stick_x;
        this.right_stick_y = gamepad.right_stick_y;
        this.dpad_up = gamepad.dpad_up;
        this.dpad_down = gamepad.dpad_down;
        this.dpad_left = gamepad.dpad_left;
        this.dpad_right = gamepad.dpad_right;
        this.a = gamepad.a;
        this.b = gamepad.b;
        this.x = gamepad.x;
        this.y = gamepad.y;
        this.guide = gamepad.guide;
        this.start = gamepad.start;
        this.back = gamepad.back;
        this.left_bumper = gamepad.left_bumper;
        this.right_bumper = gamepad.right_bumper;
        this.left_stick_button = gamepad.left_stick_button;
        this.right_stick_button = gamepad.right_stick_button;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;
    }

    /**
     * Used to un-modified joystick axis values
     * @param axis pass the axis to be returned
     * @return the raw value of the axis [-1.27, 1.28]
     */
    public double getRawAxis(AXIS axis){
        switch (axis){
            case AXIS_LEFT_STICK_X:
                return left_stick_x;
            case AXIS_LEFT_STICK_Y:
                return left_stick_y;
            case AXIS_RIGHT_STICK_X:
                return right_stick_x;
            case AXIS_RIGHT_STICK_Y:
                return right_stick_y;
            case AXIS_LEFT_TRIGGER:
                return left_trigger;
            case AXIS_RIGHT_TRIGGER:
                return right_trigger;
            default:
                return 0.0;
        }
    }

//    /**
//     * Used to read linear joystick axis from [-1, 1]
//     * @param axis the axis to be returned
//     * @return the scaled linear axis from [-1, 1]
//     */
//    public double getScaledLinearAxis(AXIS axis){
//        switch (axis){
//            case AXIS_LEFT_STICK_X:
//                return MathUtil.scaleClipped(left_stick_x);
//            case AXIS_LEFT_STICK_Y:
//                return -MathUtil.scaleClipped(left_stick_y);
//            case AXIS_RIGHT_STICK_X:
//                return MathUtil.scaleClipped(right_stick_x);
//            case AXIS_RIGHT_STICK_Y:
//                return -MathUtil.scaleClipped(right_stick_y);
//            case AXIS_LEFT_TRIGGER:
//                return MathUtil.scaleClipped(left_trigger);
//            case AXIS_RIGHT_TRIGGER:
//                return MathUtil.scaleClipped(right_trigger);
//            default:
//                return 0.0;
//        }
//    }
//
//    /**
//     * Reads a parabolic scaled joystick axis from [-1, 1]
//     * @param axis the axis to be returned
//     * @return the parabolic axis from [-1,1]
//     */
//    public double getSquaredAxis(AXIS axis){
//        switch (axis){
//            case AXIS_LEFT_STICK_X:
//                return RobotUtil.calcThrottleParabola(left_stick_x);
//            case AXIS_LEFT_STICK_Y:
//                return -RobotUtil.calcThrottleParabola(left_stick_y);
//            case AXIS_RIGHT_STICK_X:
//                return RobotUtil.calcThrottleParabola(right_stick_x);
//            case AXIS_RIGHT_STICK_Y:
//                return -RobotUtil.calcThrottleParabola(right_stick_y);
//            case AXIS_LEFT_TRIGGER:
//                return RobotUtil.calcThrottleParabola(left_trigger);
//            case AXIS_RIGHT_TRIGGER:
//                return RobotUtil.calcThrottleParabola(right_trigger);
//            default:
//                return 0.0;
//        }
//    }
//
//    /**
//     * Reads a parabolic scaled joystick axis from [-1, 1]
//     * @param axis the axis to be returned
//     * @return the parabolic axis from [-0.8,0.8]
//     */
//    public double getLimitedSquaredAxis(AXIS axis){
//        switch (axis){
//            case AXIS_LEFT_STICK_X:
//                return RobotUtil.calcLimitedThrottleParabola(left_stick_x);
//            case AXIS_LEFT_STICK_Y:
//                return -RobotUtil.calcLimitedThrottleParabola(left_stick_y);
//            case AXIS_RIGHT_STICK_X:
//                return RobotUtil.calcLimitedThrottleParabola(right_stick_x);
//            case AXIS_RIGHT_STICK_Y:
//                return -RobotUtil.calcLimitedThrottleParabola(right_stick_y);
//            case AXIS_LEFT_TRIGGER:
//                return RobotUtil.calcLimitedThrottleParabola(left_trigger);
//            case AXIS_RIGHT_TRIGGER:
//                return RobotUtil.calcLimitedThrottleParabola(right_trigger);
//            default:
//                return 0.0;
//        }
//    }

    /**
     * Checks if ANY button or axis value has changed from last update
     * @return true if any state has changed, false if all values are the same
     */
    public boolean getHasStateChanged(){
        if(left_stick_x != prev_left_stick_x)
            return true;
        if(left_stick_y != prev_left_stick_y)
            return true;
        if(right_stick_x != prev_right_stick_x)
            return true;
        if(right_stick_y != prev_right_stick_y)
            return true;
        if(dpad_up != prev_dpad_up)
            return true;
        if(dpad_down != prev_dpad_down)
            return true;
        if(dpad_left != prev_dpad_left)
            return true;
        if(dpad_right != prev_dpad_right)
            return true;
        if(a != prev_a)
            return true;
        if(b != prev_b)
            return true;
        if(x != prev_x)
            return true;
        if(y != prev_y)
            return true;
        if(guide != prev_guide)
            return true;
        if(start != prev_start)
            return true;
        if(back != prev_back)
            return true;
        if(left_bumper != prev_left_bumper)
            return true;
        if(right_bumper != prev_right_bumper)
            return true;
        if(left_stick_button != prev_left_stick_button)
            return true;
        if(right_stick_button != prev_right_stick_button)
            return true;
        if(left_trigger != prev_left_trigger)
            return true;
        if(right_trigger != prev_right_trigger)
            return true;

        return false;

    }

    /**
     * Compares the current value of any button or axis to the previously updated value of the button or axis
     * @param button the button or axis to be checked
     * @return true if the button or axis changed, false if it has stayed the same
     */
    public boolean getHasStateChanged(BUTTON button){
        switch (button){
            case DPAD_UP:
                return dpad_up != prev_dpad_up;
            case DPAD_DOWN:
                return dpad_down != prev_dpad_down;
            case DPAD_LEFT:
                return dpad_left != prev_dpad_left;
            case DPAD_RIGHT:
                return dpad_right != prev_dpad_right;
            case BUTTON_A:
                return a != prev_a;
            case BUTTON_B:
                return b != prev_b;
            case BUTTON_X:
                return x != prev_x;
            case BUTTON_Y:
                return y != prev_y;
            case BUTTON_GUIDE:
                return guide != prev_guide;
            case BUTTON_START:
                return start != prev_start;
            case BUTTON_BACK:
                return back != prev_back;
            case BUTTON_LEFT_BUMPER:
                return left_bumper != prev_left_bumper;
            case BUTTON_RIGHT_BUMPER:
                return right_bumper != prev_right_bumper;
            case BUTTON_LEFT_STICK_BUTTON:
                return left_stick_button != prev_left_stick_button;
            case BUTTON_RIGHT_STICK_BUTTON:
                return right_stick_button != prev_right_stick_button;
            default:
                return false;
        }
    }

    /**
     *
     * @param axis the axis to be checked
     * @return has the value of the axis changed
     */
    public boolean getHasStateChanged(AXIS axis){
        switch (axis){
            case AXIS_LEFT_STICK_X:
                return left_stick_x != prev_left_stick_x;
            case AXIS_LEFT_STICK_Y:
                return left_stick_y != prev_left_stick_y;
            case AXIS_RIGHT_STICK_X:
                return right_stick_x != prev_right_stick_x;
            case AXIS_RIGHT_STICK_Y:
                return right_stick_y != prev_right_stick_y;
            case AXIS_LEFT_TRIGGER:
                return left_trigger != prev_left_trigger;
            case AXIS_RIGHT_TRIGGER:
                return right_trigger != prev_right_trigger;
            default:
                return false;
        }
    }


    /**
     * A latch or toggle functionality for all buttons and axes
     * @param button the button or axis to be checked
     * @return true if the previous state was not pressed and the current state is pressed, false otherwise
     */
    public boolean getHasButtonToggled(BUTTON button){
        switch (button) {
            case DPAD_UP:
                return dpad_up && !prev_dpad_up;
            case DPAD_DOWN:
                return dpad_down && !prev_dpad_down;
            case DPAD_LEFT:
                return dpad_left && !prev_dpad_left;
            case DPAD_RIGHT:
                return dpad_right && !prev_dpad_right;
            case BUTTON_A:
                return a && !prev_a;
            case BUTTON_B:
                return b && !prev_b;
            case BUTTON_X:
                return x && !prev_x;
            case BUTTON_Y:
                return y && !prev_y;
            case BUTTON_GUIDE:
                return guide && !prev_guide;
            case BUTTON_START:
                return start && !prev_start;
            case BUTTON_BACK:
                return back && !prev_back;
            case BUTTON_LEFT_BUMPER:
                return left_bumper && !prev_left_bumper;
            case BUTTON_RIGHT_BUMPER:
                return right_bumper && !prev_right_bumper;
            case BUTTON_LEFT_STICK_BUTTON:
                return left_stick_button && !prev_left_stick_button;
            case BUTTON_RIGHT_STICK_BUTTON:
                return right_stick_button && !prev_right_stick_button;
            default:
                return false;
        }
    }

    /**
     *
     * @param axis the axis to be read as a button
     * @return was the value previously outside the button threshold and now is considered pressed
     */

    public boolean getHasButtonToggled(AXIS axis){
        switch (axis){
            case AXIS_LEFT_STICK_X:
                return ((Math.abs(left_stick_x) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_left_stick_x) < AXIS_AS_BUTTON_THRESHHOLD));
            case AXIS_LEFT_STICK_Y:
                return ((Math.abs(left_stick_y) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_left_stick_y) < AXIS_AS_BUTTON_THRESHHOLD));
            case AXIS_RIGHT_STICK_X:
                return ((Math.abs(right_stick_x) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_right_stick_x) < AXIS_AS_BUTTON_THRESHHOLD));
            case AXIS_RIGHT_STICK_Y:
                return ((Math.abs(right_stick_y) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_right_stick_y) < AXIS_AS_BUTTON_THRESHHOLD));
            case AXIS_LEFT_TRIGGER:
                return ((Math.abs(left_trigger) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_left_trigger) < AXIS_AS_BUTTON_THRESHHOLD));
            case AXIS_RIGHT_TRIGGER:
                return ((Math.abs(right_trigger) >= AXIS_AS_BUTTON_THRESHHOLD) && (Math.abs(prev_right_trigger) < AXIS_AS_BUTTON_THRESHHOLD));
            default:
                return false;
        }
    }


    /**
     * @param axis1 one axis to be checked
     * @param axis2 the other axis to be checked
     * @return true if one of the buttons has toggled and the other is pressed
     */
    public boolean  getHaveButtonsToggled(AXIS axis1, AXIS axis2){
        return (getHasButtonToggled(axis1) && getAxisAsButton(axis2)) || (getHasButtonToggled(axis2) && getAxisAsButton(axis1));
    }

    /**
     * Reads any axis as a boolean button, checking if axis value is greater than threshold value
     * @param axis the axis to be checked
     * @return true if absolute value of the axis is greater than the threshold, false otherwise
     */
    public boolean getAxisAsButton(AXIS axis){
        switch (axis) {
            case AXIS_LEFT_STICK_X:
                return Math.abs(left_stick_x) > AXIS_AS_BUTTON_THRESHHOLD;
            case AXIS_LEFT_STICK_Y:
                return Math.abs(left_stick_y) > AXIS_AS_BUTTON_THRESHHOLD;
            case AXIS_RIGHT_STICK_X:
                return Math.abs(right_stick_x) > AXIS_AS_BUTTON_THRESHHOLD;
            case AXIS_RIGHT_STICK_Y:
                return Math.abs(right_stick_y) > AXIS_AS_BUTTON_THRESHHOLD;
            case AXIS_LEFT_TRIGGER:
                return Math.abs(left_trigger) > AXIS_AS_BUTTON_THRESHHOLD;
            case AXIS_RIGHT_TRIGGER:
                return Math.abs(right_trigger) > AXIS_AS_BUTTON_THRESHHOLD;
            default:
                return false;
        }
    }



    /**
     * Set the AXIS_AS_BUTTON_THRESHHOLD value
     * @param val
     */
    public void setAXIS_AS_BUTTON_THRESHHOLD(double val){
        AXIS_AS_BUTTON_THRESHHOLD = Math.abs(val);
    }

    ////////////////////////////////////////////////////////////////
    // getters. setters specifically not implemented because all
    // variable states should be set using the update() method
    ////////////////////////////////////////////////////////////////


    public float getLeft_stick_x() {
        return left_stick_x;
    }

    public float getLeft_stick_y() {
        return left_stick_y;
    }

    public float getRight_stick_x() {
        return right_stick_x;
    }

    public float getRight_stick_y() {
        return right_stick_y;
    }

    public boolean getDpad_up() {
        return dpad_up;
    }

    public boolean getDpad_down() {
        return dpad_down;
    }

    public boolean getDpad_left() {
        return dpad_left;
    }

    public boolean getDpad_right() {
        return dpad_right;
    }

    public boolean getA() {
        return a;
    }

    public boolean getB() {
        return b;
    }

    public boolean getX() {
        return x;
    }

    public boolean getY() {
        return y;
    }

    public boolean getGuide() {
        return guide;
    }

    public boolean getStart() {
        return start;
    }

    public boolean getBack() {
        return back;
    }

    public boolean getLeft_bumper() {
        return left_bumper;
    }

    public boolean getRight_bumper() {
        return right_bumper;
    }

    public boolean getLeft_stick_button() {
        return left_stick_button;
    }

    public boolean getRight_stick_button() {
        return right_stick_button;
    }

    public float getLeft_trigger() {
        return left_trigger;
    }

    public float getRight_trigger() {
        return right_trigger;
    }

    public float getPrev_left_stick_x() {
        return prev_left_stick_x;
    }

    public float getPrev_left_stick_y() {
        return prev_left_stick_y;
    }

    public float getPrev_right_stick_x() {
        return prev_right_stick_x;
    }

    public float getPrev_right_stick_y() {
        return prev_right_stick_y;
    }

    public boolean getPrev_dpad_up() {
        return prev_dpad_up;
    }

    public boolean getPrev_dpad_down() {
        return prev_dpad_down;
    }

    public boolean getPrev_dpad_left() {
        return prev_dpad_left;
    }

    public boolean getPrev_dpad_right() {
        return prev_dpad_right;
    }

    public boolean getPrev_a() {
        return prev_a;
    }

    public boolean getPrev_b() {
        return prev_b;
    }

    public boolean getPrev_x() {
        return prev_x;
    }

    public boolean getPrev_y() {
        return prev_y;
    }

    public boolean getPrev_guide() {
        return prev_guide;
    }

    public boolean getPrev_start() {
        return prev_start;
    }

    public boolean getPrev_back() {
        return prev_back;
    }

    public boolean getPrev_left_bumper() {
        return prev_left_bumper;
    }

    public boolean getPrev_right_bumper() {
        return prev_right_bumper;
    }

    public boolean getPrev_left_stick_button() {
        return prev_left_stick_button;
    }

    public boolean getPrev_right_stick_button() {
        return prev_right_stick_button;
    }

    public float getPrev_left_trigger() {
        return prev_left_trigger;
    }

    public float getPrev_right_trigger() {
        return prev_right_trigger;
    }



    public boolean areGamepadSticksUsed()   {
        return left_stick_y != 0 || right_stick_y != 0;
    }

    public boolean  axisReturnsPositiveValue(AXIS axis)  {   return getRawAxis(axis) > 0;   }
}
