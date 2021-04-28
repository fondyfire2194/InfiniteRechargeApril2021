package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AxisJoystickButton extends JoystickButton {
    private final GenericHID m_joystick;
    private final int m_axisNumber;

    public AxisJoystickButton(GenericHID joystick, int axis_number) {
      super(joystick, axis_number);
      this.m_joystick= joystick;
      this.m_axisNumber = axis_number;
    }

    @Override
    public boolean get() {
      return m_joystick.getRawAxis(m_axisNumber) > 0.5;
    }
  }
