package frc.lib.util;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class MacroPad extends GenericHID {

    /** Represents a digital button on an XboxController. */
    public enum Button {
        button1(1),
        button2(2),
        button3(3),
        button4(4),
        button5(5),
        button6(6),
        button7(7),
        button8(8),
        button9(9),
        button10(10),
        button11(11),
        button12(12);

        public final int value;

        Button(int value) {
            this.value = value;
        }

    }

    /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public MacroPad(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

    /**
   * Read the value of this button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getButton1() {
    return getRawButton(Button.button1.value);
  }

  /**
   * Whether this button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getButton1Pressed() {
    return getRawButtonPressed(Button.button1.value);
  }

  /**
   * Whether this button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getButton1Released() {
    return getRawButtonReleased(Button.button1.value);
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing this button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent b1(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton1);
  }

  public boolean getButton2() {
    return getRawButton(Button.button2.value);
  }
  public boolean getButton2Pressed() {
    return getRawButtonPressed(Button.button2.value);
  }
  public boolean getButton2Released() {
    return getRawButtonReleased(Button.button2.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b2(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton2);
  }
  public boolean getButton3() {
    return getRawButton(Button.button3.value);
  }
  public boolean getButton3Pressed() {
    return getRawButtonPressed(Button.button3.value);
  }
  public boolean getButton3Released() {
    return getRawButtonReleased(Button.button3.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b3(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton3);
  }
  public boolean getButton4() {
    return getRawButton(Button.button4.value);
  }
  public boolean getButton4Pressed() {
    return getRawButtonPressed(Button.button4.value);
  }
  public boolean getButton4Released() {
    return getRawButtonReleased(Button.button4.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b4(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton4);
  }
  public boolean getButton5() {
    return getRawButton(Button.button5.value);
  }
  public boolean getButton5Pressed() {
    return getRawButtonPressed(Button.button5.value);
  }
  public boolean getButton5Released() {
    return getRawButtonReleased(Button.button5.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b5(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton5);
  }
  public boolean getButton6() {
    return getRawButton(Button.button6.value);
  }
  public boolean getButton6Pressed() {
    return getRawButtonPressed(Button.button6.value);
  }
  public boolean getButton6Released() {
    return getRawButtonReleased(Button.button6.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b6(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton6);
  }
  public boolean getButton7() {
    return getRawButton(Button.button7.value);
  }
  public boolean getButton7Pressed() {
    return getRawButtonPressed(Button.button7.value);
  }
  public boolean getButton7Released() {
    return getRawButtonReleased(Button.button7.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b7(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton7);
  }
  public boolean getButton8() {
    return getRawButton(Button.button8.value);
  }
  public boolean getButton8Pressed() {
    return getRawButtonPressed(Button.button8.value);
  }
  public boolean getButton8Released() {
    return getRawButtonReleased(Button.button8.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b8(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton8);
  }
  public boolean getButton9() {
    return getRawButton(Button.button9.value);
  }
  public boolean getButton9Pressed() {
    return getRawButtonPressed(Button.button9.value);
  }
  public boolean getButton9Released() {
    return getRawButtonReleased(Button.button9.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b9(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton9);
  }
  public boolean getButton10() {
    return getRawButton(Button.button10.value);
  }
  public boolean getButton10Pressed() {
    return getRawButtonPressed(Button.button10.value);
  }
  public boolean getButton10Released() {
    return getRawButtonReleased(Button.button10.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b10(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton10);
  }
  public boolean getButton11() {
    return getRawButton(Button.button11.value);
  }
  public boolean getButton11Pressed() {
    return getRawButtonPressed(Button.button11.value);
  }
  public boolean getButton11Released() {
    return getRawButtonReleased(Button.button11.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b11(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton11);
  }
  public boolean getButton12() {
    return getRawButton(Button.button12.value);
  }
  public boolean getButton12Pressed() {
    return getRawButtonPressed(Button.button12.value);
  }
  public boolean getButton12Released() {
    return getRawButtonReleased(Button.button12.value);
  }
  @SuppressWarnings("MethodName")
  public BooleanEvent b12(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton12);
  }
  
}
