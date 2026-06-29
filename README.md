# RTLS-System
A basic Real-Time-Locationing-system

# System Configuration

This RTLS system consists of one UWB Tag and four UWB Anchors.
All Tag and Anchor nodes are implemented using ESP32 UWB DW3000 modules.

Reference: [Makerfabs ESP32 UWB DW3000](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000)

The four Anchors are fixed at specific positions, and their coordinates are assigned based on measured physical distances.

When the Tag moves inside the area formed by the Anchors, the RTLS system estimates the position of the Tag using UWB ranging measurements.

Although the RTLS area can be freely defined by adjusting the Anchor positions, this system is configured in a 10 m x 10 m indoor space.

