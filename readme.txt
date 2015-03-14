
Texas Instruments, Inc.

ZASA for CC2480 Release Notes

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

Version 1.4
April 23, 2008


Notices:

- For a narrative overview of the ZASA state machine and detailed code design,
  please see the eZ430-RF2480 Demonstration Kit User’s Guide.


Changes:

- Replaced: #define ZCD_NV_QUEUED_POLL_RATE 0x0025 
  with:     #define ZCD_NV_RESPONSE_POLL_RATE 0x0026
  since setting the former is not necessary to stop all End Device polling and
  the latter one is.
- Removed references to and the code utilizing the AF and ZDO API’s of the CC2480
  RPC since they are not necessary to the ZASA.
- Created this new HAL function: halSlaveReset(), which can be used by the ZASA
  initialization code as well as the SPI driver code.
- Improved the usability of the halDelay() function by splitting the one dual-use
  parameter into two different parameters.
- Improved power-savings by properly setting pin 5 of Port 2.
- Improved power-savings by properly disabling the ADC and then turning off the
  reference voltage – a necessary two-step procedure.
- Removed sample code that optionally set the CC2480 to not use its external
  crystal since this is not recommended.
- Added a new flag, appSetPollF, to indicate the need to invoke a new function,
  appSetPoll(), which allows for a reliable flow of execution for a low power
  device to go from joining, to binding, and finally to reporting with no polling.


Bug Fixes:
 
- Fixed an intermittent bug in the SPI driver code that waits on the CC2480
  “ready” handshake in the RPC protocol.
- Fixed an incorrect call in main() to low power mode 1 by using the recommended
  call to the HAL low power mode macro.


Known Issues:

- None.

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

For technical support please contact:

Texas Instruments, Inc.
Low Power Wireless
support@ti.com
