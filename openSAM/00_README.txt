Installation
============
- Remove the SAM plugin
- If installed remove the "SAM Seasons emulator" plugin. It's functionality is included in openSAM
- Install this zip in "Custom Scenery", NOT in "Resources\plugins"
- This file should be "...\Custom Scenery\openSAM\00_README.txt" after correct installation.

Usage
=====
The openSAM plugin defines commands

"Dock jetway"   -> openSAM/dock_jwy
"Undock jetway" -> openSAM/undock_jwy
"Toggle jetway" -> openSAM/toggle_jwy

The first two comamnds are accessible through the menu as well.

In addition it exports a dataref
"opensam/jetway/status" with values:

    0 = no jetway
    1 = jetway present, available for docking
    2 = docked
   -1 = can't dock or in jetway is in transit
