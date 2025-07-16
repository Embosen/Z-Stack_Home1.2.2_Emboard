# Z-Stack Home 1.2.2 Emboard

This repository contains the Z-Stack Home 1.2.2 Emboard project, which is a ZigBee development framework for Texas Instruments CC2530 and CC2538 microcontrollers.

## Project Structure

- **Components/**: Contains all the Z-Stack components including:
  - **bsp/**: Board Support Package
  - **driverlib/**: Hardware driver libraries
  - **hal/**: Hardware Abstraction Layer
  - **mac/**: MAC layer implementation
  - **mt/**: Monitor Test interface
  - **osal/**: Operating System Abstraction Layer
  - **services/**: Additional services (ECC, SADDR, SDATA)
  - **stack/**: Z-Stack core components (AF, NWK, SAPI, SEC, SYS, ZCL, ZDO)
  - **usblib/**: USB library for CC2538
  - **zmac/**: ZMAC implementation

- **Projects/**: Contains sample applications and tools:
  - **zstack/HomeAutomation/**: Home automation sample applications
  - **Libraries/**: Pre-compiled libraries for different platforms
  - **OTA/**: Over-The-Air update components
  - **Tools/**: Development tools and utilities
  - **ZAP/**: Z-Stack Application Profile samples
  - **ZMain/**: Main application entry points
  - **ZNP/**: ZigBee Network Processor implementations

- **Tools/**: Additional development tools including Z-Tool

## Supported Platforms

- CC2530
- CC2531
- CC2538
- MSP5438

## Getting Started

1. Install IAR Embedded Workbench for 8051 (for CC2530/CC2531) or ARM (for CC2538)
2. Open the appropriate project file (.eww) in the Projects directory
3. Configure your target platform and build settings
4. Build and flash the application

## Sample Applications

- **Em_Sensor_A**: Energy measurement sensor application
- **HA-SampleLight**: Home automation light control sample
- **HA-SampleSwitch**: Home automation switch sample
- **OTA_Dongle**: Over-the-air update dongle application

## Documentation

For detailed documentation, refer to the Texas Instruments Z-Stack documentation and the TI-RTOS documentation.

## License

This project is based on Texas Instruments Z-Stack Home 1.2.2. Please refer to the original TI license terms.

## Contributing

This is a reference implementation. For modifications, please ensure compatibility with the Z-Stack framework and test thoroughly on your target hardware. 