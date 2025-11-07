// package org.frc3512.robot.subsystems.led;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// public class LedIOReal implements LedIO {

//   private AddressableLED led;
//   private AddressableLEDBuffer buffer;

//   public LedIOReal() {

//     led = new AddressableLED(0);
//     buffer = new AddressableLEDBuffer(35);
//   }

//   @Override
//   public void updateInputs(LedIOInputs inputs) {
//     led.setData(buffer);
//     led.start();

//     inputs.ledColor = buffer.getLED(1).toString();
//   }

//   @Override
//   public void setPattern(LedStates state) {
//     state.pattern.applyTo(buffer);
//   }
// }
