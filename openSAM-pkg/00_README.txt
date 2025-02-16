============
Installation
============
- Remove the SAM plugin (but keep the embedded SAM_Library somewhere)
- If you already deleted SAM but use a SAM replacement library (e.g. "FlyAgi - SAM Fallback Library") remove it.
- If installed remove the "SAM Seasons emulator" plugin. Its functionality is included in openSAM.
- From within this zip install
    "openSAM_Library" into "Custom Scenery"
    "openSAM"         into "Resources/plugins"

Some sceneries (e.g. for Zero Dollar Payware, Taimodels) require the original SAM_Library
- If not already done link or copy the SAM_library into "Custom Scenery"
- Make sure openSAM_Library is above SAM_Library in scenery_packs.ini

=====
Usage
=====
The openSAM plugin defines commands:

"Dock jetway"   -> openSAM/dock_jwy
"Undock jetway" -> openSAM/undock_jwy
"Toggle UI"     -> openSAM/toggle_ui
"Toggle jetway" -> openSAM/toggle_jwy

The first three commands are accessible through the menu as well.

openSAM augments XP12's standard command "sim/ground_ops/jetway":
    - if there is a SAM jetway operate it
    - otherwise try to operate a XP12 standard jetway (if there is none it's a noop)

In addition openSAM exports datarefs:

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
--------------------------------------------------
openSAM calls optional commands "openSAM/post_dock" and "openSAM/pre_undock".
See a sample in the lua directory for the ToLiss fleet and more examples in the contributed directory.

Activation of DGS
-----------------
After you LAND (= some air time!) on an airport the plugin activates and searches actively for
suitable stands in the direction that you are taxiing. Note that you MUST have your beacon on.
Once you come closer the VDGS or the Marshaller give appropriate guidance information.

If you just want to try out the feature without prior flight you must issue the
command "openSAM/activate", "Manually activate searching for DGS" through a binding or the menu.
Beacon on, of course!

That should look familiar to users of AutoDGS.

Multiplayer support
------------------
openSAM supports
 - xPilot
 - Traffic Global XP
 - LiveTraffic

Set up your multiplayer environment as needed (e.g. connect to Vatsim and/or enable/disable MP plugins).
Then select "Toggle Multiplayer Support" in openSAM's menu.
If xPilot is connected to Vatsim that takes precedence over other installed multplayer plugins.
The menu text of "Toggle Multiplayer Support" shows which personality is enabled.

Note:
xPilot and liveTraffic receive aircraft positions from other sceneries / simulators or the real world and these
may not match up with your installed scenery. Your mileage will vary.

TGXP gives the visually most pleasing rendition.

The problem of a second door
----------------------------
Up to now there is no reliable way to determine the position of a second door.
Therefore door positions must be maintained in the config file "acf_door_position.txt".

To find the position proceed as follows:

Locate the "openSAM: plane loaded" line in Log.txt.

openSAM: plane loaded: B742, plane_cg_y: -2.44, plane_cg_z: 31.81, door 1: x: -2.93, y: 1.77, z: -22.45

Use these values as a starting point: door 2 is somewhat behind meaning z towards the tail and the
correct line to enter in "acf_door_position.txt" is:

B742 2 -2.93 1.77 -14.40

Please share results so they can be included in future updates.

=====================================================================
"Zero configuration Marshaller and VDGS service" for scenery creators
=====================================================================
This is for sceneries with no or XP12 default jetways that should be equipped with Marshallers or VDGS.
In WED just place the appropriate assets from 'openSAM' in the library pane into the scenery.
SAM_Library is not required.
Then copy file "openSAM_Library/zero_config_dgs/sam.xml" into your scenery and you are done.

=============================================================
"Zero configuration SAM library jetways" for scenery creators
=============================================================
In case XP12 default jetways are not sufficient SAM library jetways can be used with zero configuration.
SAM_Library in required.
Just place them with proper initial orientation of the tunnel in WED. At runtime the cabin will point
perpendicular to the stand with slight randown variations.
Use of SAM's authoring tool is not necessary.
Then copy file "openSAM_Library/zero_config_dgs/sam.xml" into your scenery and you are done.
