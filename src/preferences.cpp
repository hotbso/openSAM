//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cstdio>
#include <filesystem>

#include "opensam.h"
#include "my_plane.h"
#include "seasons.h"
#include "autodgs_airport.h"
#include "log_msg.h"

#include "pugixml.hpp"

static void ApplySeasonPreferences() {
    Seasons::auto_season = prefs.auto_season;
    Seasons::season = prefs.season;
    Seasons::nh = prefs.north_hemisphere;
}

static bool TryLoadXmlPreferences(const std::string& path) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());
    if (!result) {
        LogMsg("Could not load XML preferences '%s': %s", path.c_str(), result.description());
        return false;
    }

    pugi::xml_node root = doc.child("preferences");
    pugi::xml_node seasons = root.child("seasons");
    pugi::xml_node ui = root.child("ui");
    if (root.empty() || seasons.empty() || ui.empty()) {
        LogMsg("XML preferences '%s' do not have the expected structure", path.c_str());
        return false;
    }

    prefs.auto_jetway_select = root.attribute("auto_jetway_select").as_bool(prefs.auto_jetway_select);
    prefs.manage_xp12_jetways = root.attribute("manage_xp12_jetways").as_bool(prefs.manage_xp12_jetways);
    prefs.auto_enable_mp = root.attribute("auto_enable_mp").as_bool(prefs.auto_enable_mp);
    prefs.default_vdgs_type = root.attribute("default_vdgs_type").as_int(prefs.default_vdgs_type);
    prefs.auto_season = seasons.attribute("auto_season").as_int(prefs.auto_season);
    prefs.season = seasons.attribute("season").as_int(prefs.season);
    prefs.north_hemisphere = seasons.attribute("north_hemisphere").as_bool(prefs.north_hemisphere);
    prefs.ui_left = ui.attribute("left").as_int(prefs.ui_left);
    prefs.ui_top = ui.attribute("top").as_int(prefs.ui_top);
    prefs.ui_right = ui.attribute("right").as_int(prefs.ui_right);
    prefs.ui_bottom = ui.attribute("bottom").as_int(prefs.ui_bottom);

    LogMsg("Loaded preferences from '%s'", path.c_str());
    return true;
}

static bool TryLoadLegacyPreferences(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (f == nullptr) {
        LogMsg("Could not open legacy preferences '%s'; using defaults", path.c_str());
        return false;
    }

    int auto_jetway_select = prefs.auto_jetway_select;
    int manage_xp12_jetways = prefs.manage_xp12_jetways;
    int n = fscanf(f, "%i,%i,%i,%i,%i,%i,%i,%i,%i", &prefs.auto_season, &prefs.season, &auto_jetway_select,
                   &prefs.default_vdgs_type, &prefs.ui_left, &prefs.ui_top, &prefs.ui_right, &prefs.ui_bottom,
                   &manage_xp12_jetways);
    fclose(f);

    if (n != 9) {
        LogMsg("Legacy preferences '%s' are incomplete; using defaults", path.c_str());
        return false;
    }

    prefs.auto_jetway_select = auto_jetway_select != 0;
    prefs.manage_xp12_jetways = manage_xp12_jetways != 0;
    if (prefs.season < 0) {
        prefs.north_hemisphere = false;
        prefs.season = -prefs.season;
    }

    prefs.auto_enable_mp = false; // default value for legacy preferences
    LogMsg("Loaded legacy preferences from '%s'", path.c_str());
    return true;
}

void LoadPrefs() {
    const std::string xml_path = user_cfg_dir + "preferences.xml";
    if (!TryLoadXmlPreferences(xml_path))
        TryLoadLegacyPreferences(user_cfg_dir + "prefs.prf");

    ApplySeasonPreferences();
    LogMsg(
        "Preferences: auto_season: %d, season: %d, north_hemisphere: %d, auto_select_jws: %d, default_vdgs_type: %d, "
        "ui:(%d, %d, %d, %d), manage_xp12_jetways: %d",
        prefs.auto_season, prefs.season, prefs.north_hemisphere, prefs.auto_jetway_select, prefs.default_vdgs_type, prefs.ui_left,
        prefs.ui_top, prefs.ui_right, prefs.ui_bottom, prefs.manage_xp12_jetways);
}

void SavePrefs() {
    prefs.auto_jetway_select = my_plane->auto_mode();
    prefs.auto_season = Seasons::auto_season;
    prefs.season = Seasons::season;
    prefs.north_hemisphere = Seasons::nh;

    pugi::xml_document doc;
    pugi::xml_node root = doc.append_child("preferences");
    root.append_attribute("auto_jetway_select") = prefs.auto_jetway_select;
    root.append_attribute("manage_xp12_jetways") = prefs.manage_xp12_jetways;
    root.append_attribute("auto_enable_mp") = prefs.auto_enable_mp;
    root.append_attribute("default_vdgs_type") = prefs.default_vdgs_type;

    pugi::xml_node seasons = root.append_child("seasons");
    seasons.append_attribute("auto_season") = prefs.auto_season;
    seasons.append_attribute("season") = prefs.season;
    seasons.append_attribute("north_hemisphere") = prefs.north_hemisphere;

    pugi::xml_node ui = root.append_child("ui");
    ui.append_attribute("left") = prefs.ui_left;
    ui.append_attribute("top") = prefs.ui_top;
    ui.append_attribute("right") = prefs.ui_right;
    ui.append_attribute("bottom") = prefs.ui_bottom;

    const std::string path = user_cfg_dir + "preferences.xml";
    if (!doc.save_file(path.c_str(), "  ", pugi::format_default | pugi::format_indent_attributes)) {
        LogMsg("Failed to save preferences to '%s'", path.c_str());
        return;
    }

    LogMsg("Saved preferences to '%s'", path.c_str());
}
