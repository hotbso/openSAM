--[[
    openSAM: open source SAM emulator for X Plane

    Copyright (C) 2024  Holger Teutsch

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA

]]

function openSAM_post_dock()
    logMsg("openSAM_post_dock called")

    -- for ToLiss open door 1 + ext power on + chocks
    if PLANE_ICAO == "A319" or PLANE_ICAO == "A20N" or PLANE_ICAO == "A321" or PLANE_ICAO == "A346" or PLANE_ICAO == "A339" then
        set_array("AirbusFBW/PaxDoorModeArray", 0, 2)
        set("AirbusFBW/EnableExternalPower", 1)
        set("AirbusFBW/Chocks", 1)
        return
    end

end

function openSAM_pre_undock()
    logMsg("openSAM_pre_undock")

    -- for ToLiss ensure doors are closed
    if PLANE_ICAO == "A319" or PLANE_ICAO == "A20N" or PLANE_ICAO == "A321" or PLANE_ICAO == "A346" or PLANE_ICAO == "A339" then
        set_array("AirbusFBW/PaxDoorModeArray", 0, 0)
        set_array("AirbusFBW/PaxDoorModeArray", 2, 0)
        set_array("AirbusFBW/PaxDoorModeArray", 6, 0)
        return
    end
end


-- create the commands
create_command( "openSAM/post_dock", "openSAM_post_dock",
                "",
                "",
                "openSAM_post_dock()")

add_macro("openSAM_post_dock", "openSAM_post_dock()")

create_command( "openSAM/pre_undock", "openSAM_pre_undock",
                "",
                "",
                "openSAM_pre_undock()")

add_macro("openSAM_pre_undock", "openSAM_pre_undock()")
