Installation
============
- Remove the SAM plugin
- If you already deleted SAM but use a SAM replacement library (e.g. "FlyAgi - SAM Fallback Library") remove it.
- If installed remove the "SAM Seasons emulator" plugin. It's functionality is included in openSAM.
- From within this zip install
    "openSAM_Library" into "Custom Scenery"
    "openSAM"         into "Resources/plugins"

If you must keep the SAM_Library (e.g. for Zero Dollar Payware freeware)
- If not already done link the SAM_library into "Custom Scenery"
- Make sure openSAM is above SAM_Library in scenery_packs.ini
- Within openSAM copy "library-with_SAM_Library.txt" to "library.txt"

Usage
=====
The openSAM plugin defines commands:

"Dock jetway"   -> openSAM/dock_jwy
"Undock jetway" -> openSAM/undock_jwy
"Toggle jetway" -> openSAM/toggle_jwy

The first two comamnds are accessible through the menu as well.

In addition it exports a dataref
"opensam/jetway/status" with values:

    0 = no jetway
    1 = jetway present, available for docking
    2 = docked
   -1 = can't dock or jetway is in transit

The problem of a second door
============================
Up to now there is no reliable way to determine the position of a second door. 
Therefore door positions must be maintained in the config file "acf_door_position.txt".

To find the position proceed as follows:

Locate the "openSAM: plane loaded" line in Log.txt.

openSAM: plane loaded: B742, plane_cg_y: -2.44, plane_cg_z: 31.81, door 1: x: -2.93, y: 1.77, z: -22.45

Use these values as a starting point: door 2 is somewhat behind meaning z towards the tail and the
correct line to enter in "acf_door_position.txt" is:

B742 2 -2.93 1.77 -14.40

Please share results so they can be included in future updates.
