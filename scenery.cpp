//
//    openSAM: open source SAM emulator for X Plane
//
//    Copyright (C) 2024, 2025  Holger Teutsch
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

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <unordered_map>

#include "pugiconfig.hpp"
#include "pugixml.hpp"

#include "scenery.h"
#include "dgs/apt_airport.h"
#include "quadtree.h"
#include "quadtree.inl"
#include "opensam.h"
#include "samjw.h"
#include "os_anim.h"

std::vector<Scenery> Scenery::sceneries;

static int LookupDrf(const std::string& name) {
    for (unsigned int i = 0; i < SamDrf::sam_drfs.size(); i++)
        if (SamDrf::sam_drfs[i].name == name)
            return i;

    return -1;
}

static int LookupObj(const Scenery* sc, const std::string& id) {
    for (unsigned int i = 0; i < sc->sam_objs_.size(); i++)
        if (sc->sam_objs_[i].id == id)
            return i;

    return -1;
}

static void ParseDatarefs(const pugi::xml_node& sc_node) {
    pugi::xml_node datarefs = sc_node.child("datarefs");
    if (datarefs.empty())
        return;

    for (pugi::xml_node dataref : datarefs.children("dataref")) {
        const char *name = dataref.attribute("name").as_string(nullptr);
        if (name == nullptr) {
            LogMsg("dataref with missing name attribute ignored");
            continue;
        }

        if (LookupDrf(name) >= 0) {
            LogMsg("duplicate definition for dataref '%s', ignored", name);
            continue;
        }

        SamDrf drf;
        drf.name = name;
        drf.autoplay = dataref.attribute("autoplay").as_bool(false);
        drf.randomize_phase = dataref.attribute("randomize_phase").as_bool(false);
        drf.augment_wind_speed = dataref.attribute("augment_wind_speed").as_bool(false);

        drf.t.reserve(10);
        drf.v.reserve(10);
        drf.s.reserve(10);

        for (pugi::xml_node anim : dataref.children("animation")) {
            float t = anim.attribute("t").as_float(0.0f);
            float v = anim.attribute("v").as_float(0.0f);
            if (drf.t.size() > 0 && t == drf.t.back())  // no double entries
                drf.v.back() = v;       // just overwrite with recent value
            else {
                int n = drf.t.size();
                drf.t.push_back(t);
                drf.v.push_back(v);
                // save a few cycles in the accessor
                float s = n > 0 ? (v - drf.v[n - 1]) / (t - drf.t[n - 1]) : 0.0f;
                drf.s.push_back(s);
            }
        }

        drf.n_tv = drf.t.size();
        SamDrf::sam_drfs.push_back(std::move(drf));
    }
}

static void ParseObjects(const pugi::xml_node& sc_node, Scenery* sc) {
    pugi::xml_node objects = sc_node.child("objects");
    if (objects.empty())
        return;

    for (pugi::xml_node instance : objects.children("instance")) {
        SamObj obj;
        const char* id = instance.attribute("id").as_string(nullptr);
        if (id == nullptr) {
            LogMsg("instance with missing id attribute ignored");
            continue;
        }
        obj.id = instance.attribute("id").as_string("");
        obj.latitude = instance.attribute("latitude").as_float(0.0f);
        obj.longitude = instance.attribute("longitude").as_float(0.0f);
        obj.elevation = instance.attribute("elevation").as_float(0.0f);
        obj.heading = instance.attribute("heading").as_float(0.0f);
        LogMsg("Parsed object: id='%s', lat=%0.6f, lon=%0.6f, elev=%0.2f, heading=%0.2f", obj.id.c_str(), obj.latitude, obj.longitude, obj.elevation, obj.heading);
        sc->sam_objs_.push_back(std::move(obj));
    }
}

static void ParseGui(const pugi::xml_node& sc_node, Scenery* sc) {
    pugi::xml_node gui = sc_node.child("gui");
    if (gui.empty())
        return;

    for (pugi::xml_node checkbox : gui.children("checkbox")) {
        SamAnim anim;
        anim.label = checkbox.attribute("label").as_string("");
        anim.title = checkbox.attribute("title").as_string("");

        anim.obj_idx = anim.drf_idx = -1;

        const char* inst = checkbox.attribute("instance").as_string(nullptr);
        if (inst)
            anim.obj_idx = LookupObj(sc, inst);

        const char* name = checkbox.attribute("dataref").as_string(nullptr);
        if (name)
            anim.drf_idx = LookupDrf(name);

        if (anim.obj_idx >= 0 && anim.drf_idx >= 0)
            sc->sam_anims_.push_back(std::move(anim));
        else {
            LogMsg("dataref of object not found for checkbox entry");
        }
    }
}

static void ParseJetways(const pugi::xml_node& sc_node) {
    pugi::xml_node jetways = sc_node.child("jetways");
    if (jetways.empty())
        return;

    for (pugi::xml_node jetway : jetways.children("jetway")) {
        SamJw* jw = new SamJw();
        jw->is_lib_jw_inst = jetway.attribute("libraryInstance").as_bool(false);
        jw->name = jetway.attribute("name").as_string("");
        jw->latitude = jetway.attribute("latitude").as_float(0.0f);
        jw->longitude = jetway.attribute("longitude").as_float(0.0f);
        jw->heading = jetway.attribute("heading").as_float(0.0f);
        jw->height = jetway.attribute("height").as_float(0.0f);
        jw->wheelPos = jetway.attribute("wheelPos").as_float(0.0f);
        jw->cabinPos = jetway.attribute("cabinPos").as_float(0.0f);
        jw->cabinLength = jetway.attribute("cabinLength").as_float(0.0f);
        jw->wheelDiameter = jetway.attribute("wheelDiameter").as_float(0.0f);
        jw->wheelDistance = jetway.attribute("wheelDistance").as_float(0.0f);
        jw->sound = jetway.attribute("sound").as_string("");
        jw->minRot1 = jetway.attribute("minRot1").as_float(0.0f);
        jw->maxRot1 = jetway.attribute("maxRot1").as_float(0.0f);
        jw->minRot2 = jetway.attribute("minRot2").as_float(0.0f);
        jw->maxRot2 = jetway.attribute("maxRot2").as_float(0.0f);
        jw->minRot3 = jetway.attribute("minRot3").as_float(0.0f);
        jw->maxRot3 = jetway.attribute("maxRot3").as_float(0.0f);
        jw->minExtent = jetway.attribute("minExtent").as_float(0.0f);
        jw->maxExtent = jetway.attribute("maxExtent").as_float(0.0f);
        jw->minWheels = jetway.attribute("minWheels").as_float(0.0f);
        jw->maxWheels = jetway.attribute("maxWheels").as_float(0.0f);
        jw->initialRot1 = jetway.attribute("initialRot1").as_float(0.0f);
        jw->initialRot2 = jetway.attribute("initialRot2").as_float(0.0f);
        jw->initialRot3 = jetway.attribute("initialRot3").as_float(0.0f);
        jw->initialExtent = jetway.attribute("initialExtent").as_float(0.0f);
        const char* door_loc = jetway.attribute("forDoorLocation").as_string(nullptr);
        if (door_loc) {
            if (0 == strcmp(door_loc, "LF2"))
                jw->door = 1;
            else if (0 == strcmp(door_loc, "LU1"))
                jw->door = 2;
        }
        jw->base_name = jw->name;  // for later use when we fabricate names for zero config jetways

        // sanitize all heading values entering the plugin in order to avoid stalls in fem::RA
        jw->heading = fmodf(jw->heading, 360.0f);

        jw->ComputeBbox();

        // simple sanity check, e.g Aerosoft LEBL has bogus values
        if (is_between(jw->latitude, -85.0, 85.0) && is_between(jw->longitude, -180.0, 180.0))
            sam_jw_list.push_back(jw);
        else {
            LogMsg("Jetway with invalid lat,lon: %0.6f, %0.6f ignored", jw->latitude, jw->longitude);
            delete (jw);
        }
    }
}

static void ParseLibraryJetways(const pugi::xml_node& sc_node, std::unordered_map<std::string, SamLibJw*>& lib_jw_map) {
    pugi::xml_node sets = sc_node.child("sets");
    if (sets.empty())
        return;

    for (pugi::xml_node set : sets.children("set")) {
        SamLibJw* ljw = new SamLibJw;
        ljw->id = set.attribute("id").as_string("");
        ljw->name = set.attribute("name").as_string("");
        ljw->height = set.attribute("height").as_float(0.0f);
        ljw->wheelPos = set.attribute("wheelPos").as_float(0.0f);
        ljw->cabinPos = set.attribute("cabinPos").as_float(0.0f);
        ljw->cabinLength = set.attribute("cabinLength").as_float(0.0f);
        ljw->wheelDiameter = set.attribute("wheelDiameter").as_float(0.0f);
        ljw->wheelDistance = set.attribute("wheelDistance").as_float(0.0f);
        ljw->minRot1 = set.attribute("minRot1").as_float(0.0f);
        ljw->maxRot1 = set.attribute("maxRot1").as_float(0.0f);
        ljw->minRot2 = set.attribute("minRot2").as_float(0.0f);
        ljw->maxRot2 = set.attribute("maxRot2").as_float(0.0f);
        ljw->minRot3 = set.attribute("minRot3").as_float(0.0f);
        ljw->maxRot3 = set.attribute("maxRot3").as_float(0.0f);
        ljw->minExtent = set.attribute("minExtent").as_float(0.0f);
        ljw->maxExtent = set.attribute("maxExtent").as_float(0.0f);
        ljw->minWheels = set.attribute("minWheels").as_float(0.0f);
        ljw->maxWheels = set.attribute("maxWheels").as_float(0.0f);
        lib_jw_map[ljw->id] = ljw;
    }
}

static bool ParseSamXml(const std::string& fn, std::unordered_map<std::string, SamLibJw*>& lib_jw_map, Scenery* sc) {
    pugi::xml_document doc;

    // Load from disk:
    pugi::xml_parse_result result = doc.load_file(fn.c_str());
    if (!result)
        return false;

    pugi::xml_node lib = doc.child("libraryjetwayconfiguration");
    if (!lib.empty()) {
        ParseLibraryJetways(lib, lib_jw_map);
        LogMsg("Parsing library jetways from '%s'", fn.c_str());
        return true; // for now
    }

    pugi::xml_node sc_node = doc.child("scenery");
    if (sc_node.empty()) {
        LogMsg("No <scenery> element found in '%s'", fn.c_str());
        return false;
    }

    ParseDatarefs(sc_node);
    ParseLibraryJetways(sc_node, lib_jw_map);

    if (sc == nullptr)
        return true;

    sc->name_ = sc_node.attribute("name").as_string("bad");
    ParseObjects(sc_node, sc);
    ParseGui(sc_node, sc);
    ParseJetways(sc_node);
    return true;
}

// SceneryPacks constructor
SceneryPacks::SceneryPacks(const std::string& xp_dir) {
    std::string scpi_name(xp_dir + "/Custom Scenery/scenery_packs.ini");

    std::ifstream scpi(scpi_name);
    if (scpi.fail()) {
        LogMsg("Can't open '%s'", scpi_name.c_str());
        return;
    }

    sc_paths.reserve(500);
    std::string line;

    while (std::getline(scpi, line)) {
        if (line.empty())
            continue;

        if (line.back() == '\r')
            line.pop_back();

        if (!line.starts_with("SCENERY_PACK ") || line.contains("*GLOBAL_AIRPORTS*"))
            continue;

        line.erase(0, 13);
        if (line.empty())   // be paranoid
            continue;

        std::string sc_path;
        bool is_absolute = (line[0] == '/' || line.contains(':'));
        if (is_absolute)
            sc_path = line;
        else
            sc_path = xp_dir + "/" + line;

        // posixify
        for (unsigned i = 0; i < sc_path.size(); i++)
            if (sc_path[i] == '\\')
                sc_path[i] = '/';

        // autoortho pretends every file exists but
        // reads give errors. And likely XPME isn't better.
        if (line.contains("/z_ao_") || line.contains("/XPME_"))
            continue;

        if (sc_path.contains("/openSAM_Library/")) {
            openSAM_Library_path = sc_path;
            continue;
        }

        if (sc_path.contains("/SAM_Library/")) {
            SAM_Library_path = sc_path;
            continue;
        }

        sc_paths.push_back(sc_path);
    }

    scpi.close();
    sc_paths.shrink_to_fit();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// collect all sceneries
void Scenery::CollectSceneries(const SceneryPacks& scp, int& max_sam_stands) {
    max_sam_stands = 0;
    std::unordered_map<std::string, SamLibJw*> lib_jw_map;

    // drefs from openSAM_Library must come first
    if (scp.openSAM_Library_path.empty() || !ParseSamXml(scp.openSAM_Library_path + "sam.xml", lib_jw_map, nullptr))
        throw std::runtime_error("openSAM_Library is not installed or inaccessible!");

    if (!scp.SAM_Library_path.empty()) {
        if (!ParseSamXml(scp.SAM_Library_path + "libraryjetways.xml", lib_jw_map, nullptr))
            LogMsg("Warning: SAM_Library is installed but 'SAM_Library/libraryjetways.xml' could not be processed");
    }

    sceneries.reserve(scp.sc_paths.size());
    sam_jw_list.reserve(1000);  // avoid too many reallocations, usually there are much more stands than jetways

    for (auto& sc_path : scp.sc_paths) {
        ParseSamXml(sc_path + "libraryjetways.xml", lib_jw_map, nullptr);  // always try libraryjetways.xml

        Scenery sc;
        sc.jw_idx_end_ = sc.jw_idx_start_ = sam_jw_list.size();
        std::string sam_xml_pathname = sc_path + "sam.xml";
        bool is_opensam = ParseSamXml(sam_xml_pathname, lib_jw_map, &sc);
        sc.jw_idx_end_ = sam_jw_list.size();

        // read stands from apt.dat
        int n_stands = 0;

        dgs::AptAirport* apt = nullptr;
        if (is_opensam) {
            // will be used with openSAM personality
            apt = dgs::AptAirport::ParseAptDat(sc_path + "Earth nav data/apt.dat", /* ignore */ false,
                                               /* filter_autodgs */ false, n_stands);
            if (apt) {
                apt->is_opensam_ = true;
                apt->sam_xml_pathname_ = sam_xml_pathname;
            }
        } else {
            // will be used with AutoDGS personality
            bool ignore = (std::filesystem::exists(sc_path + "no_autodgs") ||
                           std::filesystem::exists(sc_path + "no_autodgs.txt"));
            apt = dgs::AptAirport::ParseAptDat(sc_path + "Earth nav data/apt.dat", ignore, /* filter_autodgs */ true,
                                               n_stands);
        }

        if (!(apt && is_opensam))
            continue;

        // don't save empty sceneries
        if (n_stands == 0 && sc.sam_anims_.empty())
            continue;

        max_sam_stands = std::max(max_sam_stands, n_stands);

        // shrink to actual
        sc.sam_anims_.shrink_to_fit();
        sc.sam_objs_.shrink_to_fit();

        sceneries.emplace_back(std::move(sc));
    }

    sceneries.shrink_to_fit();
    SamDrf::sam_drfs.shrink_to_fit();

    // transfer collected library jetways to vector for fast access by dref acessors
    lib_jw.reserve(lib_jw_map.size() + 1);
    lib_jw.push_back(nullptr);  // must start at 1 due to logic in the dref accessors

    for (const auto& [id_str, sam_lib_jw] : lib_jw_map)
        lib_jw.push_back(sam_lib_jw);

    lib_jw.shrink_to_fit();

    // load the quadtree with all jetways from all sceneries for fast lookup by position in the dref accessors
    for (auto jw : sam_jw_list) {
        //LogMsg("Inserting jetway '%s' at %0.6f, %0.6f into quadtree", jw->repr().c_str(), jw->lat(), jw->lon());
        jw_quadtree.Insert(jw);
        //jw_quadtree.Dump();
    }

    LogMsg("Finished collecting sceneries, total jetways in quadtree: %u", (unsigned)jw_quadtree.size());
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Update the <jetways> section of a scenery's sam.xml with the current jetway configuration, return whether successful
bool UpdateSamXml(const std::vector<SamJw*> lib_jw_instances, const std::string& xml_pathname) {
    pugi::xml_document doc;

    // Load from disk:
    pugi::xml_parse_result result = doc.load_file(xml_pathname.c_str(), pugi::parse_default | pugi::parse_comments);
    if (!result)
        return false;

    pugi::xml_node sc_node = doc.child("scenery");
    if (sc_node.empty()) {
        LogMsg("No <scenery> element found in '%s'", xml_pathname.c_str());
        return false;
    }

    // openSAM never used docks
    pugi::xml_node docks = sc_node.child("docks");
    if (!docks.empty())
        sc_node.remove_child(docks);

    pugi::xml_node jetways = sc_node.child("jetways");
    if (jetways.empty())
        jetways = sc_node.append_child("jetways");
    else {
        // remove all jetways with libraryInstance="true" from sam.xml
        for (pugi::xml_node child = jetways.first_child(); child;) {
            // Advance to next_sibling() BEFORE removing current child
            pugi::xml_node next = child.next_sibling();
            if (child.type() == pugi::node_element && std::string_view(child.name()) == "jetway" &&
                child.attribute("libraryInstance").as_bool())
                jetways.remove_child(child);

            child = next;
        }
    }

    // add configured library jetway instances
    for (auto jw : lib_jw_instances) {
        if (jw->is_lib_jw_inst && !jw->is_zc_jw) {
            pugi::xml_node jw_node = jetways.append_child("jetway");
            jw_node.append_attribute("libraryInstance") = true;
            jw_node.append_attribute("name") = jw->name.c_str();
            jw_node.append_attribute("latitude") = jw->latitude;
            jw_node.append_attribute("longitude") = jw->longitude;
            jw_node.append_attribute("heading") = jw->heading;
            jw_node.append_attribute("initialRot1") = jw->initialRot1;
            jw_node.append_attribute("initialRot2") = jw->initialRot2;
            jw_node.append_attribute("initialRot3") = jw->initialRot3;
            jw_node.append_attribute("initialExtent") = jw->initialExtent;
        }
    }

    std::ofstream xml(xml_pathname, std::ios::out | std::ios::trunc);
    if (!xml.is_open()) {
        LogMsg("Failed to open '%s' for writing", xml_pathname.c_str());
        return false;
    }

    doc.save(xml, "  ", pugi::format_default | pugi::format_indent_attributes);
    xml.close();
    return true;
}
