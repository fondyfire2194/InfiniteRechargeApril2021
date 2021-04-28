// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class POVBBox {
    private ButtonBox m_box;

    public POVBBox(ButtonBox box) {

        m_box = box;
    }

    // JOYSTICK DPAD HAT METHODS
    /**
     * @return The current "angle" from the DPad (POV switch)
     */
    public int DPad() {
        return m_box.getPOV(0);
    }

    /**
     * @return True if the DPad is pushed right, False if it is not pressed
     */
    public boolean DPadRight() {
        if ((m_box.getPOV(0) >= 315 || m_box.getPOV(0) <= 45) && m_box.getPOV(0) != -1)
            return true;
        else
            return false;
    }

    /**
     * @return True if the DPad is pushed down, False if it is not pressed
     */
    public boolean DPadDown() {
        if (m_box.getPOV(0) >= 45 && m_box.getPOV(0) <= 135)
            return true;
        else
            return false;
    }

    /**
     * @return True if the DPad is pushed left, False if it is not pressed
     */
    public boolean DPadLeft() {
        if (m_box.getPOV(0) >= 135 && m_box.getPOV(0) <= 225)
            return true;
        else
            return false;
    }

    /**
     * @return True if the DPad is pushed up, False if it is not pressed
     */
    public boolean DPadUp() {
        if (m_box.getPOV(0) >= 225 && m_box.getPOV(0) <= 315)
            return true;
        else
            return false;
    }

}
