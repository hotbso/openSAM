Installation
============
- Remove the SAM plugin (but keep the embedded SAM_Library somewhere)
- If you already deleted SAM but use a SAM replacement library (e.g. "FlyAgi - SAM Fallback Library") remove it.
- If installed remove the "SAM Seasons emulator" plugin. Its functionality is included in openSAM.
- From within this zip install
    "openSAM_Library" into "Custom Scenery"
    "openSAM"         into "Resources/plugins"

If you must keep the SAM_Library (e.g. for Zero Dollar Payware freeware)
- If not already done link or copy the SAM_library into "Custom Scenery"
- Make sure openSAM_Library is above SAM_Library in scenery_packs.ini
- Within openSAM_Library copy "library-with_SAM_Library.txt" to "library.txt"

Usage
=====
The openSAM plugin defines commands:

"Dock jetway"   -> openSAM/dock_jwy
"Undock jetway" -> openSAM/undock_jwy
"Toggle jetway" -> openSAM/toggle_jwy

The first two commands are accessible through the menu as well.

In addition it exports datarefs:

"opensam/jetway/number"
    number of jetways at stand

"opensam/jetway/status"
    0 = no jetway(s)
    1 = jetway(s) present, available for docking
    2 = docked
   -1 = can't dock or jetway(s) in transit

"opensam/jetway/door/status" array per door
    0 = no jetway docked at this door
    1 = jetway docked at this door


Customize actions after docking / before undocking
==================================================
openSAM calls optional commands "openSAM/post_dock" and "openSAM/pre_undock".
See a sample in the lua directory for the ToLiss fleet and more examples in the contributed directory.

Activation of DGS
=================
After you LAND (= some air time!) on an airport the plugin activates and searches actively for
suitable stands in the direction that you are taxiing. Note that you MUST have your beacon on.
Once you come closer the VDGS or the Marshaller give appropriate guidance information.

If you just want to try out the feature without prior flight you must issue the
command "openSAM/activate", "Manually activate searching for DGS" through a binding or the menu.
Beacon on, of course!

That should look familiar to users of AutoDGS.

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
