package frc.lib.util;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.hal.DriverStationJNI;

public class MacroPad extends GenericHID {

  private Button lastInput;

  /** Represents a digital button on an macropad. */
  public enum Button {
    button1(1, 1),
    button2(1, 2),
    button3(1, 3),
    button4(2, 1),
    button5(2, 2),
    button6(2, 3),
    button7(3, 1),
    button8(3, 2),
    button9(3, 3),
    button10(4, 1),
    button11(4, 2),
    button12(4, 3);

    public final int column;
    public final int row;

    Button(int row, int column) {
      this.row = row;
      this.column = column;
    }

    public int value() {
      return Integer.parseInt(this.name().replaceFirst("button", ""));
    }

  }

  private Set<Button> buttons = new HashSet<>();

  public Set<Button> getButtons() {
    Set<Button> pressedButtons = new HashSet<Button>();
    int buttons = DriverStation.getStickButtons(2);
    int m = 1;
    for (int i = 0; i < Button.values().length; i++) {
      if ((buttons & m) == m) {
        pressedButtons.add(Button.values()[i]);
      }
      m <<= 1;
    }
    return pressedButtons;
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is
   *             plugged into.
   */
  public MacroPad(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  public Button getLast() {
    return lastInput;
  }

  /**
   * Read the value of this button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getButton1() {
    return getRawButton(Button.button1.value());
  }

  /**
   * Whether this button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getButton1Pressed() {
    return getRawButtonPressed(Button.button1.value());
  }

  /**
   * Whether this button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getButton1Released() {
    return getRawButtonReleased(Button.button1.value());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing this button's digital signal attached
   *         to the given
   *         loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent b1(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton1);
  }

  public boolean getButton2() {
    return getRawButton(Button.button2.value());
  }

  public boolean getButton2Pressed() {
    return getRawButtonPressed(Button.button2.value());
  }

  public boolean getButton2Released() {
    return getRawButtonReleased(Button.button2.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b2(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton2);
  }

  public boolean getButton3() {
    return getRawButton(Button.button3.value());
  }

  public boolean getButton3Pressed() {
    return getRawButtonPressed(Button.button3.value());
  }

  public boolean getButton3Released() {
    return getRawButtonReleased(Button.button3.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b3(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton3);
  }

  public boolean getButton4() {
    return getRawButton(Button.button4.value());
  }

  public boolean getButton4Pressed() {
    return getRawButtonPressed(Button.button4.value());
  }

  public boolean getButton4Released() {
    return getRawButtonReleased(Button.button4.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b4(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton4);
  }

  public boolean getButton5() {
    return getRawButton(Button.button5.value());
  }

  public boolean getButton5Pressed() {
    return getRawButtonPressed(Button.button5.value());
  }

  public boolean getButton5Released() {
    return getRawButtonReleased(Button.button5.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b5(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton5);
  }

  public boolean getButton6() {
    return getRawButton(Button.button6.value());
  }

  public boolean getButton6Pressed() {
    return getRawButtonPressed(Button.button6.value());
  }

  public boolean getButton6Released() {
    return getRawButtonReleased(Button.button6.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b6(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton6);
  }

  public boolean getButton7() {
    return getRawButton(Button.button7.value());
  }

  public boolean getButton7Pressed() {
    return getRawButtonPressed(Button.button7.value());
  }

  public boolean getButton7Released() {
    return getRawButtonReleased(Button.button7.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b7(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton7);
  }

  public boolean getButton8() {
    return getRawButton(Button.button8.value());
  }

  public boolean getButton8Pressed() {
    return getRawButtonPressed(Button.button8.value());
  }

  public boolean getButton8Released() {
    return getRawButtonReleased(Button.button8.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b8(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton8);
  }

  public boolean getButton9() {
    return getRawButton(Button.button9.value());
  }

  public boolean getButton9Pressed() {
    return getRawButtonPressed(Button.button9.value());
  }

  public boolean getButton9Released() {
    return getRawButtonReleased(Button.button9.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b9(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton9);
  }

  public boolean getButton10() {
    return getRawButton(Button.button10.value());
  }

  public boolean getButton10Pressed() {
    return getRawButtonPressed(Button.button10.value());
  }

  public boolean getButton10Released() {
    return getRawButtonReleased(Button.button10.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b10(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton10);
  }

  public boolean getButton11() {
    return getRawButton(Button.button11.value());
  }

  public boolean getButton11Pressed() {
    return getRawButtonPressed(Button.button11.value());
  }

  public boolean getButton11Released() {
    return getRawButtonReleased(Button.button11.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b11(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton11);
  }

  public boolean getButton12() {
    return getRawButton(Button.button12.value());
  }

  public boolean getButton12Pressed() {
    return getRawButtonPressed(Button.button12.value());
  }

  public boolean getButton12Released() {
    return getRawButtonReleased(Button.button12.value());
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b12(EventLoop loop) {
    return new BooleanEvent(loop, this::getButton12);
  }

}
