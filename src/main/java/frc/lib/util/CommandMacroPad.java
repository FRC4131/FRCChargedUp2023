package frc.lib.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* A subclass of {@link XboxController} with {@link Trigger} factories for command-based.
*
* @see XboxController
*/
@SuppressWarnings("MethodName")
public class CommandMacroPad extends CommandGenericHID {
  private final MacroPad m_hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is
   *             plugged into.
   */
  public CommandMacroPad(int port) {
    super(port);
    m_hid = new MacroPad(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public MacroPad getHID() {
    return m_hid;
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @return an event instance representing this button's digital signal
   *         attached to the {@link
   *         CommandScheduler#getDefaultButtonLoop() default scheduler button
   *         loop}.
   * @see #leftBumper(EventLoop)
   */
  public Trigger b1() {
    return b1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal
   *         attached to the given
   *         loop.
   */
  public Trigger b1(EventLoop loop) {
    return m_hid.b1(loop).castTo(Trigger::new);
  }


  public Trigger b2() {
    return b2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b2(EventLoop loop) {
    return m_hid.b2(loop).castTo(Trigger::new);
  }
  public Trigger b3() {
    return b3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b3(EventLoop loop) {
    return m_hid.b3(loop).castTo(Trigger::new);
  }
  public Trigger b4() {
    return b4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b4(EventLoop loop) {
    return m_hid.b4(loop).castTo(Trigger::new);
  }
  public Trigger b5() {
    return b5(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b5(EventLoop loop) {
    return m_hid.b5(loop).castTo(Trigger::new);
  }
  public Trigger b6() {
    return b6(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b6(EventLoop loop) {
    return m_hid.b6(loop).castTo(Trigger::new);
  }
  public Trigger b7() {
    return b7(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b7(EventLoop loop) {
    return m_hid.b7(loop).castTo(Trigger::new);
  }
  public Trigger b8() {
    return b8(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b8(EventLoop loop) {
    return m_hid.b8(loop).castTo(Trigger::new);
  }
  public Trigger b9() {
    return b9(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b9(EventLoop loop) {
    return m_hid.b9(loop).castTo(Trigger::new);
  }
  public Trigger b10() {
    return b10(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b10(EventLoop loop) {
    return m_hid.b10(loop).castTo(Trigger::new);
  }

    public Trigger b11() {
    return b11(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b11(EventLoop loop) {
    return m_hid.b11(loop).castTo(Trigger::new);
  }
    public Trigger b12() {
    return b12(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger b12(EventLoop loop) {
    return m_hid.b12(loop).castTo(Trigger::new);
  }

}