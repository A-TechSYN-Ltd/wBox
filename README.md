# wbox v2.0.0
# Records navigation data & raw data for PPK

Software Version is v2.0.0 and Revision Version 1.3

Bug Fixes:
  Long-term logging disconnection issue has been resolved.

Added Features:
  Validation structure added to System Configuration.
  Failsafe mode added.
  A diagnostic system that logs outputs to a file has been implemented.
  Access to the device's serial number and software version information has been added.
  PPK (Post-Processed Kinematic) support added.
  Logging and operation frequencies have been updated.
  ITOW (Time Of Week) variable has been added.

Knowing Issues:

  While the module operates at 10 Hz, with the current software version, the NAV-PVT message is mostly received every 100 ms, but occasionally the interval extends to 200 ms. Therefore, in the previous version, the system was configured to process data only when it     
  arrived, in order to avoid timing-related issues.
  In addition to this, the system configuration has been updated as follows:
  
  Module running at 10 Hz + NAV-PVT logging at 10 Hz (previous version) →
  Module running at 5 Hz + NAV-PVT logging at 5 Hz + RAWX logging at 5 Hz + SFRBX logging at 1 Hz (v2.0.0)
