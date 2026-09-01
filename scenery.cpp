//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
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

std::vector<Scenery*> Scenery::sceneries_;

static const char* jetways_comment =
    " model_id == \"\" means this jetway is a library instance, otherwise it is a local model instance ";

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
        // LogMsg("Parsed object: id='%s', lat=%0.6f, lon=%0.6f, elev=%0.2f, heading=%0.2f", obj.id.c_str(), obj.latitude, obj.longitude, obj.elevation, obj.heading);
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

// sam1 legacy format
static void SamParseJetways(const pugi::xml_node& sc_node) {
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
        jw->wheel_pos = jetway.attribute("wheelPos").as_float(0.0f);
        jw->cabin_pos = jetway.attribute("cabinPos").as_float(0.0f);
        jw->cabin_length = jetway.attribute("cabinLength").as_float(0.0f);
        jw->wheel_diameter = jetway.attribute("wheelDiameter").as_float(0.0f);
        jw->wheel_distance = jetway.attribute("wheelDistance").as_float(0.0f);
        jw->sound = jetway.attribute("sound").as_string("");
        jw->min_rot1 = jetway.attribute("minRot1").as_float(-90.0f);
        jw->max_rot1 = jetway.attribute("maxRot1").as_float(90.0f);
        jw->min_rot2 = jetway.attribute("minRot2").as_float(-5.0f);
        jw->max_rot2 = jetway.attribute("maxRot2").as_float(90.0f);
        jw->min_rot3 = jetway.attribute("minRot3").as_float(-6.0f);
        jw->max_rot3 = jetway.attribute("maxRot3").as_float(6.0f);
        jw->min_extent = jetway.attribute("minExtent").as_float(0.0f);
        jw->max_extent = jetway.attribute("maxExtent").as_float(0.0f);
        jw->initial_rot1 = jetway.attribute("initialRot1").as_float(0.0f);
        jw->initial_rot2 = jetway.attribute("initialRot2").as_float(0.0f);
        jw->initial_rot3 = jetway.attribute("initialRot3").as_float(0.0f);
        jw->initial_extent = jetway.attribute("initialExtent").as_float(0.0f);
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

// opensam.xml format
static void OpenSamParseJetways(const pugi::xml_node& sc_node, Scenery* sc) {
    pugi::xml_node jetways = sc_node.child("jetways");
    if (jetways.empty())
        return;

    for (pugi::xml_node jetway : jetways.children("jetway")) {
        SamJw* jw = new SamJw();
        jw->name = jetway.attribute("name").as_string("");
        jw->model_id = jetway.attribute("model_id").as_string("");
        jw->latitude = jetway.attribute("latitude").as_float(0.0f);
        jw->longitude = jetway.attribute("longitude").as_float(0.0f);
        jw->heading = jetway.attribute("heading").as_float(0.0f);
        jw->door = jetway.attribute("door").as_int(0);
        jw->initial_rot1 = jetway.attribute("min_rot1").as_float(-90.0f);
        jw->initial_rot2 = jetway.attribute("max_rot1").as_float(90.0f);
        jw->initial_rot1 = jetway.attribute("initial_rot1").as_float(0.0f);
        jw->initial_rot2 = jetway.attribute("initial_rot2").as_float(0.0f);
        jw->initial_rot3 = jetway.attribute("initial_rot3").as_float(0.0f);
        jw->initial_extent = jetway.attribute("initial_extent").as_float(0.0f);

        if (jw->model_id.empty())
            jw->is_lib_jw_inst = true;
        else {
            auto it = sc->jw_models_.find(jw->model_id);
            if (it == sc->jw_models_.end()) {
                LogMsg("Jetway '%s' with unknown model_id '%s' ignored", jw->name.c_str(), jw->model_id.c_str());
                delete jw;
                continue;
            }

            const SamJwModel* jwm = it->second;
            jw->model_id = jwm->model_id;
            jw->height = jwm->height;
            jw->wheel_pos = jwm->wheel_pos;
            jw->cabin_pos = jwm->cabin_pos;
            jw->cabin_length = jwm->cabin_length;

            jw->wheel_diameter = jwm->wheel_diameter;
            jw->wheel_distance = jwm->wheel_distance;

            jw->min_rot1 = -90.0f;
            jw->max_rot1 = 90.0f;

            jw->min_rot2 = jwm->min_rot2;
            jw->max_rot2 = jwm->max_rot2;

            jw->min_rot3 = jwm->min_rot3;
            jw->max_rot3 = jwm->max_rot3;

            jw->min_extent = jwm->min_extent;
            jw->max_extent = jwm->max_extent;
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

// legacy sam.xml format has <sets> and <set> elements, which are equivalent to the <jetwayDescriptions> and <description> elements in the new format. The following functions parse both formats and store the data in SamJwModel structures.
static void ParseJetwaySet(const pugi::xml_node& set_node, SamJwModel* jwm) {
    jwm->model_id = set_node.attribute("id").as_string("");
    jwm->name = set_node.attribute("name").as_string("");
    jwm->height = set_node.attribute("height").as_float(0.0f);
    jwm->wheel_pos = set_node.attribute("wheelPos").as_float(0.0f);
    jwm->cabin_pos = set_node.attribute("cabinPos").as_float(0.0f);
    jwm->cabin_length = set_node.attribute("cabinLength").as_float(0.0f);
    jwm->wheel_diameter = set_node.attribute("wheelDiameter").as_float(0.0f);
    jwm->wheel_distance = set_node.attribute("wheelDistance").as_float(0.0f);
    jwm->min_rot2 = set_node.attribute("minRot2").as_float(-90.0f);
    jwm->max_rot2 = set_node.attribute("maxRot2").as_float(90.0f);
    jwm->min_rot3 = set_node.attribute("minRot3").as_float(-8.0f);
    jwm->max_rot3 = set_node.attribute("maxRot3").as_float(8.0f);
    jwm->min_extent = set_node.attribute("minExtent").as_float(0.0f);
    jwm->max_extent = set_node.attribute("maxExtent").as_float(0.0f);
}

// Add a <local_jw> or <library_jw> element to the <models> section of a scenery's opensam.xml file
static void AddModel(pugi::xml_node models, const SamJwModel* ljw, const char* model_tag) {
    pugi::xml_node model = models.append_child(model_tag);
    model.append_attribute("model_id") = ljw->model_id.c_str();
    model.append_attribute("name") = ljw->name.c_str();
    model.append_attribute("height") = ljw->height;
    model.append_attribute("wheel_pos") = ljw->wheel_pos;
    model.append_attribute("cabin_pos") = ljw->cabin_pos;
    model.append_attribute("cabin_length") = ljw->cabin_length;
    model.append_attribute("wheel_diameter") = ljw->wheel_diameter;
    model.append_attribute("wheel_distance") = ljw->wheel_distance;
    model.append_attribute("min_rot2") = ljw->min_rot2;
    model.append_attribute("max_rot2") = ljw->max_rot2;
    model.append_attribute("min_rot3") = ljw->min_rot3;
    model.append_attribute("max_rot3") = ljw->max_rot3;
    model.append_attribute("min_extent") = ljw->min_extent;
    model.append_attribute("max_extent") = ljw->max_extent;
}

// opensam.xml
static void ParseJetwayModel(const pugi::xml_node& set_node, SamJwModel* jwm) {
    jwm->model_id = set_node.attribute("model_id").as_string("");
    jwm->name = set_node.attribute("name").as_string("");
    jwm->height = set_node.attribute("height").as_float(0.0f);
    jwm->wheel_pos = set_node.attribute("wheel_pos").as_float(0.0f);
    jwm->cabin_pos = set_node.attribute("cabin_pos").as_float(0.0f);
    jwm->cabin_length = set_node.attribute("cabin_length").as_float(0.0f);
    jwm->wheel_diameter = set_node.attribute("wheel_diameter").as_float(0.0f);
    jwm->wheel_distance = set_node.attribute("wheel_distance").as_float(0.0f);
    jwm->min_rot2 = set_node.attribute("min_rot2").as_float(-90.0f);
    jwm->max_rot2 = set_node.attribute("max_rot2").as_float(90.0f);
    jwm->min_rot3 = set_node.attribute("min_rot3").as_float(-8.0f);
    jwm->max_rot3 = set_node.attribute("max_rot3").as_float(8.0f);
    jwm->min_extent = set_node.attribute("min_extent").as_float(0.0f);
    jwm->max_extent = set_node.attribute("max_extent").as_float(0.0f);
}

// Parse <sets> and <set> elements from scenery XML and store them in the lib_jw_map
static void ParseSets(const pugi::xml_node& sc_node, std::unordered_map<std::string, SamJwModel*>& lib_jw_map) {
    const pugi::xml_node sets = sc_node.child("sets");
    if (sets.empty())
        return;

    for (const pugi::xml_node set : sets.children("set")) {
        SamJwModel* ljw = new SamJwModel;
        ParseJetwaySet(set, ljw);
        lib_jw_map[ljw->model_id] = ljw;
    }
}

static void ParseJwDescriptions(const pugi::xml_node& sc_node, Scenery& sc) {
    const pugi::xml_node descriptions = sc_node.child("jetwayDescriptions");
    if (descriptions.empty())
        return;

    for (const pugi::xml_node description : descriptions.children("description")) {
        SamJwModel* ljw = new SamJwModel;
        ParseJetwaySet(description, ljw);
        sc.jw_models_[ljw->model_id] = ljw;
    }
}

// parse and convert legacy sam.xml format to opensam.xml format
static bool ConvertSamXml(const std::string& fn, const std::string& opensam_xml_pathname,
                        std::unordered_map<std::string, SamJwModel*>& lib_jw_map, Scenery* sc) {
    pugi::xml_document doc;

    // Load from disk:
    pugi::xml_parse_result result = doc.load_file(fn.c_str());
    if (!result)
        return false;

    LogMsg("Processing '%s'", fn.c_str());
    pugi::xml_node lib = doc.child("libraryjetwayconfiguration");
    if (!lib.empty()) {
        ParseSets(lib, lib_jw_map);
        LogMsg("Parsing library jetways from '%s'", fn.c_str());
        return true;  // for now
    }

    pugi::xml_node sc_node = doc.child("scenery");
    if (sc_node.empty()) {
        LogMsg("No <scenery> element found in '%s'", fn.c_str());
        return false;
    }

    ParseDatarefs(sc_node);
    ParseSets(sc_node, lib_jw_map);

    if (sc == nullptr)
        return true;

    sc->name_ = sc_node.attribute("name").as_string("no name");
    ParseObjects(sc_node, sc);
    ParseGui(sc_node, sc);
    ParseJwDescriptions(sc_node, *sc);  // save them as models for this scenery, not in the global lib_jw_map
    sc->jw_idx_end_ = sc->jw_idx_start_ = sam_jw_list.size();
    SamParseJetways(sc_node);
    sc->jw_idx_end_ = sam_jw_list.size();

    int iauto = 1;
    for (int i = sc->jw_idx_start_; i < sc->jw_idx_end_; i++) {
        SamJw* jw = sam_jw_list[i];
        if (jw->is_lib_jw_inst)
            continue;

        if (jw->model_id.empty()) {
            static constexpr float eps = 0.05f;  // allow for some rounding errors in the sam.xml values
            for (const auto& [_, jwm] : sc->jw_models_) {
                if (std::abs(jw->height - jwm->height) < eps && std::abs(jw->wheel_pos - jwm->wheel_pos) < eps &&
                    std::abs(jw->cabin_pos - jwm->cabin_pos) < eps &&
                    std::abs(jw->cabin_length - jwm->cabin_length) < eps &&
                    std::abs(jw->wheel_diameter - jwm->wheel_diameter) < eps &&
                    std::abs(jw->wheel_distance - jwm->wheel_distance) < eps &&
                    std::abs(jw->min_extent - jwm->min_extent) < eps &&
                    std::abs(jw->max_extent - jwm->max_extent) < eps) {
                    jw->model_id = jwm->model_id;
                    break;
                }
            }
            if (jw->model_id.empty()) {
                // create a model
                SamJwModel* jwm = new SamJwModel;
                jwm->model_id = "auto_" + std::to_string(iauto++);
                jwm->name = "Auto-generated for " + jw->base_name;
                jwm->height = jw->height;
                jwm->wheel_pos = jw->wheel_pos;
                jwm->cabin_pos = jw->cabin_pos;
                jwm->cabin_length = jw->cabin_length;
                jwm->wheel_diameter = jw->wheel_diameter;
                jwm->wheel_distance = jw->wheel_distance;
                jwm->min_rot2 = jw->min_rot2;
                jwm->max_rot2 = jw->max_rot2;
                jwm->min_rot3 = jw->min_rot3;
                jwm->max_rot3 = jw->max_rot3;
                jwm->min_extent = jw->min_extent;
                jwm->max_extent = jw->max_extent;
                sc->jw_models_[jwm->model_id] = jwm;
                jw->model_id = jwm->model_id;
            }
        }
    }

    // openSAM never used docks or static_aircrafts, so remove them from the scenery XML
    pugi::xml_node docks = sc_node.child("docks");
    if (!docks.empty())
        sc_node.remove_child(docks);

    pugi::xml_node static_aircrafts = sc_node.child("static_aircrafts");
    if (!static_aircrafts.empty())
        sc_node.remove_child(static_aircrafts);

    pugi::xml_node descriptions = sc_node.child("jetwayDescriptions");
    if (!descriptions.empty())
        sc_node.remove_child(descriptions);

    pugi::xml_node models = sc_node.child("models");
    if (!models.empty())
        sc_node.remove_child(models);

    models = sc_node.append_child("models");


    for (const auto& [_, ljw] : sc->jw_models_)
        AddModel(models, ljw, "local_jw");

    pugi::xml_node sets = sc_node.child("sets");
    if (!sets.empty()) {
        // move and reformat as library_jw models in the <models> section
        for (pugi::xml_node set : sets.children("set")) {
            auto it = lib_jw_map.find(set.attribute("id").as_string(""));
            if (it == lib_jw_map.end())  // should never happen, but be paranoid
                continue;
            SamJwModel* ljw = it->second;
            AddModel(models, ljw, "library_jw");
        }

        sc_node.remove_child(sets);
    }

    pugi::xml_node jetways = sc_node.child("jetways");
    if (!jetways.empty())
        sc_node.remove_child(jetways);

    jetways = sc_node.append_child("jetways");
    jetways.append_child(pugi::node_comment).set_value(jetways_comment);

    // ... and rewrite them in the new format
    for (int i = sc->jw_idx_start_; i < sc->jw_idx_end_; i++) {
        SamJw* jw = sam_jw_list[i];

        pugi::xml_node jetway = jetways.append_child("jetway");
        jetway.append_attribute("name") = jw->name.c_str();
        if (jw->is_lib_jw_inst)
            jetway.append_attribute("model_id") = "";
        else
            jetway.append_attribute("model_id") = jw->model_id.c_str();

        jetway.append_attribute("latitude") = jw->latitude;
        jetway.append_attribute("longitude") = jw->longitude;
        jetway.append_attribute("heading") = jw->heading;
        jetway.append_attribute("min_rot1") = jw->min_rot1;
        jetway.append_attribute("max_rot1") = jw->max_rot1;
        jetway.append_attribute("initial_rot1") = jw->initial_rot1;
        jetway.append_attribute("initial_rot2") = jw->initial_rot2;
        jetway.append_attribute("initial_rot3") = jw->initial_rot3;
        jetway.append_attribute("initial_extent") = jw->initial_extent;
        if (!jw->is_lib_jw_inst)
            jetway.append_attribute("door") = jw->door;
    }

    sc_node.insert_move_before(models, jetways);  // for readability

    if (!doc.save_file(opensam_xml_pathname.c_str(), "  ", pugi::format_default | pugi::format_indent_attributes)) {
        LogMsg("Failed to save '%s'", opensam_xml_pathname.c_str());
        return false;
    }

    return true;
}

// parse opensam.xml format
static bool ParseOpenSamXml(const std::string& fn, std::unordered_map<std::string, SamJwModel*>& lib_jw_map,
                            Scenery* sc) {
    pugi::xml_document doc;

    // Load from disk:
    pugi::xml_parse_result result = doc.load_file(fn.c_str());
    if (!result)
        return false;

    pugi::xml_node lib = doc.child("libraryjetwayconfiguration");
    if (!lib.empty()) {
        ParseSets(lib, lib_jw_map);
        LogMsg("Parsing library jetways from '%s'", fn.c_str());
        return true;  // for now
    }

    LogMsg("Parsing opensam.xml from '%s'", fn.c_str());

    pugi::xml_node sc_node = doc.child("scenery");
    if (sc_node.empty()) {
        LogMsg("No <scenery> element found in '%s'", fn.c_str());
        return false;
    }

    ParseDatarefs(sc_node);
    ParseSets(sc_node, lib_jw_map);

    if (sc == nullptr)
        return true;

    sc->name_ = sc_node.attribute("name").as_string("bad");
    ParseObjects(sc_node, sc);
    ParseGui(sc_node, sc);

    pugi::xml_node models = sc_node.child("models");
    if (!models.empty()) {
        for (pugi::xml_node model : models.children("local_jw")) {
            SamJwModel* ljw = new SamJwModel;
            ParseJetwayModel(model, ljw);
            sc->jw_models_[ljw->model_id] = ljw;
        }

        for (pugi::xml_node model : models.children("library_jw")) {
            SamJwModel* ljw = new SamJwModel;
            ParseJetwayModel(model, ljw);
            lib_jw_map[ljw->model_id] = ljw;
        }
    }

    sc->jw_idx_end_ = sc->jw_idx_start_ = sam_jw_list.size();
    OpenSamParseJetways(sc_node, sc);
    sc->jw_idx_end_ = sam_jw_list.size();
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
    std::unordered_map<std::string, SamJwModel*> lib_jw_map;

    // drefs from openSAM_Library must come first
    if (scp.openSAM_Library_path.empty() || !ConvertSamXml(scp.openSAM_Library_path + "sam.xml", "", lib_jw_map, nullptr))
        throw std::runtime_error("openSAM_Library is not installed or inaccessible!");

    if (!scp.SAM_Library_path.empty()) {
        if (!ConvertSamXml(scp.SAM_Library_path + "libraryjetways.xml", "", lib_jw_map, nullptr))
            LogMsg("Warning: SAM_Library is installed but 'SAM_Library/libraryjetways.xml' could not be processed");
    }

    sceneries_.reserve(scp.sc_paths.size());
    sam_jw_list.reserve(1000);  // avoid too many reallocations, usually there are much more stands than jetways

    for (auto& sc_path : scp.sc_paths) {
        std::string opensam_xml_pathname = sc_path + "opensam.xml";
        ConvertSamXml(sc_path + "libraryjetways.xml", opensam_xml_pathname, lib_jw_map, nullptr);  // always try libraryjetways.xml

        Scenery* sc = new Scenery();
        // try to parse opensam.xml first, if not found, try legacy sam.xml
        bool is_opensam = ParseOpenSamXml(opensam_xml_pathname, lib_jw_map, sc);
        if (!is_opensam)
            is_opensam = ConvertSamXml(sc_path + "sam.xml", opensam_xml_pathname, lib_jw_map, sc);   // converts to opensam.xml

        // read stands from apt.dat
        int n_stands = 0;

        dgs::AptAirport* apt = nullptr;
        if (is_opensam) {
            // will be used with openSAM personality
            apt = dgs::AptAirport::ParseAptDat(sc_path + "Earth nav data/apt.dat", /* ignore */ false,
                                               /* filter_autodgs */ false, n_stands);
            if (apt)
                apt->is_opensam_ = true;
        } else {
            // will be used with AutoDGS personality
            bool ignore = (std::filesystem::exists(sc_path + "no_autodgs") ||
                           std::filesystem::exists(sc_path + "no_autodgs.txt"));
            apt = dgs::AptAirport::ParseAptDat(sc_path + "Earth nav data/apt.dat", ignore, /* filter_autodgs */ true,
                                               n_stands);
        }

        if (!(apt && is_opensam)) {
            delete sc;
            continue;
        }

        // don't save empty sceneries
        if (n_stands == 0 && sc->sam_anims_.empty()) {
            delete sc;
            continue;
        }

        max_sam_stands = std::max(max_sam_stands, n_stands);

        // shrink to actual
        sc->sam_anims_.shrink_to_fit();
        sc->sam_objs_.shrink_to_fit();
        sc->sam_xml_pathname_ = std::move(opensam_xml_pathname);
        sc->airport_ = apt;     // back pointer to the apt airport data for this scenery
        apt->scenery_ = sc;     // back pointer to the scenery for this apt airport data
        sceneries_.push_back(sc);
    }

    sceneries_.shrink_to_fit();
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
// Update the <jetways> section of a scenery's opensam.xml with the current jetway configuration, return whether successful
bool Scenery::UpdateOpenSamXml(const std::vector<SamJw*> jw_instances) {
    pugi::xml_document doc;

    // Load from disk:
    pugi::xml_parse_result result = doc.load_file(sam_xml_pathname_.c_str(), pugi::parse_default | pugi::parse_comments);
    if (!result)
        return false;

    pugi::xml_node sc_node = doc.child("scenery");
    if (sc_node.empty()) {
        LogMsg("No <scenery> element found in '%s'", sam_xml_pathname_.c_str());
        return false;
    }

    pugi::xml_node models = sc_node.child("models");
    if (models.empty())
        sc_node.append_child("models");

    // for now remove all models with tag "local_jw" and replace them with the current set of jw_models_
    for (pugi::xml_node child = models.first_child(); child;) {
        // Advance to next_sibling() BEFORE removing current child
        pugi::xml_node next = child.next_sibling();
        if (child.type() == pugi::node_element && 0 == strcmp(child.name(), "local_jw"))
            models.remove_child(child);

        child = next;
    }

    for (const auto& [_, ljw] : jw_models_)
        AddModel(models, ljw, "local_jw");

    // rewrite all jetways from the vector
    pugi::xml_node jetways = sc_node.child("jetways");
    if (!jetways.empty())
        sc_node.remove_child(jetways);

    jetways = sc_node.append_child("jetways");
    jetways.append_child(pugi::node_comment).set_value(jetways_comment);

    for (auto jw : jw_instances) {
        pugi::xml_node jetway = jetways.append_child("jetway");
        jetway.append_attribute("name") = jw->name.c_str();
        if (jw->is_lib_jw_inst)
            jetway.append_attribute("model_id") = "";
        else
            jetway.append_attribute("model_id") = jw->model_id.c_str();

        jetway.append_attribute("latitude") = jw->latitude;
        jetway.append_attribute("longitude") = jw->longitude;
        jetway.append_attribute("heading") = jw->heading;
        jetway.append_attribute("min_rot1") = jw->min_rot1;
        jetway.append_attribute("max_rot1") = jw->max_rot1;
        jetway.append_attribute("initial_rot1") = jw->initial_rot1;
        jetway.append_attribute("initial_rot2") = jw->initial_rot2;
        jetway.append_attribute("initial_rot3") = jw->initial_rot3;
        jetway.append_attribute("initial_extent") = jw->initial_extent;
        if (!jw->is_lib_jw_inst)
            jetway.append_attribute("door") = jw->door;
    }

    if (!doc.save_file(sam_xml_pathname_.c_str(), "  ", pugi::format_default | pugi::format_indent_attributes)) {
        LogMsg("Failed to save '%s'", sam_xml_pathname_.c_str());
        return false;
    }

    return true;
}
