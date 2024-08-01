# SimpleLEDComm_ArduinoUno_MasterSlave

This project demonstrates a low-cost, optical communication system between two Arduino Uno boards using standard, visible LEDs. The setup consists of two Arduino Uno boards, each equipped with an onboard SMD LED and an externally connected LED. At any given moment, the two LEDs on each board exhibit the same behavior: either both are OFF, both are blinking in sync, or both are ON.

The master Arduino is equipped with a push button. Pressing this button cycles the LED behavior on the master board through three modes: OFF, blinking, and ON. Invisibly to the human eye, the master communicates the current LED state to the slave Arduino using optical signals, as described in the paper "Very Low-Cost Sensing and Communication Using Bidirectional LEDs" by Mitsubishi Electric Research Laboratories.

The slave Arduino, if within optical range, receives and decodes the transmitted signal from the master and synchronizes its LEDs to match the master's state. A video demonstrating the project will be included in the repository.

This project showcases a practical application of bidirectional LED communication, highlighting how simple components can be used to achieve effective and synchronized communication between two microcontrollers.

## Project Structure

The repository is organized as follows:

- `Master/`: Contains the Arduino code for the master board.
- `Slave/`: Contains the Arduino code for the slave board.
- `media/`: Contains media files such as the [demonstration video](https://github.com/MaurizioMalaspina/SimpleLEDComm_ArduinoUno_MasterSlave/blob/main/media/LightComm%20Demo.mp4) and images.
- `docs/`: Contains documentation such as the document by MITSUBISHI ELECTRIC RESEARCH LABORATORIES.

## Dependencies

### Master

The master Arduino code depends on the `avdweb_VirtualDelay.h/.c` library for handling non-blocking delays already included in this repo. You can find the library repo [here](https://www.arduino.cc/reference/en/libraries/avdweb_virtualdelay/).

### Slave

Similarly, the slave Arduino code also depends on the `avdweb_VirtualDelay.h/.c` library for handling non-blocking delays already included in this repo. You can find the library repo [here](https://www.arduino.cc/reference/en/libraries/avdweb_virtualdelay/).

## Requirements

- 2 x Arduino Uno
- 2 x Visible LEDs (one for each board)
- 1 x Push button (for the master board)
- Standard electronic components and wiring
- `avdweb_VirtualDelay` library

## Getting Started

1. **Upload the Code**:
   - Upload the code from the `Master/` directory to the master Arduino Uno.
   - Upload the code from the `Slave/` directory to the slave Arduino Uno.

2. **Connect the Components**:
   - Connect the LEDs and push button as specified in the respective code comments.

3. **Power Up and Test**:
   - Power up both Arduino boards.
   - Press the button on the master board to cycle through the LED modes.
   - Observe the LEDs on the slave board synchronize with the master's LED state if within optical range.

## References

- ["Very Low-Cost Sensing and Communication Using Bidirectional LEDs" by Mitsubishi Electric Research Laboratories]([https://www.merl.com/publications/docs/TR2012-010.pdf](https://www.merl.com/publications/docs/TR2003-35.pdf))

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
