/// @file       LTAPI.cpp
/// @brief      LiveTraffic API
/// @details    API to access LiveTraffic's aircraft information.
///             Data transfer from LiveTraffic to your plugin is by dataRefs
///             in a fast, efficient way:
///             LiveTraffic copies data of several planes combined into
///             defined structures. LTAPI handles all that in the background
///             and provides you with an array of aircraft information with
///             numerical info like position, heading, speed and
///             textual info like type, registration, call sign, flight number.
/// @see        https://twinfan.github.io/LTAPI/
/// @author     Birger Hoppe
/// @copyright  (c) 2019-2025 Birger Hoppe
/// @copyright  Permission is hereby granted, free of charge, to any person obtaining a
///             copy of this software and associated documentation files (the "Software"),
///             to deal in the Software without restriction, including without limitation
///             the rights to use, copy, modify, merge, publish, distribute, sublicense,
///             and/or sell copies of the Software, and to permit persons to whom the
///             Software is furnished to do so, subject to the following conditions:\n
///             The above copyright notice and this permission notice shall be included in
///             all copies or substantial portions of the Software.\n
///             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
///             IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
///             FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
///             AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
///             LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
///             OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
///             THE SOFTWARE.

#include <stdio.h>
#include <cstring>
#include <algorithm>
#include <cassert>
#include "LTAPI.h"

#include "XPLMPlugin.h"

// Windows: I prefer std::min
#ifdef min
#undef min
#endif

//
// MARK: Globals
//

/// LiveTraffic's plugin signature
#define LT_PLUGIN_SIGNATURE     "TwinFan.plugin.LiveTraffic"

/// shared dataRef for accessing current aircraft under camera
constexpr const char* SDR_CAMERA_TCAS_IDX = "sim/multiplayer/camera/tcas_idx";
/// shared dataRef for accessing current aircraft under camera
constexpr const char* SDR_CAMERA_MODES_ID = "sim/multiplayer/camera/modeS_id";

// The following macros define dataRef access statically,
// then assign its current value to the passed-in variable:

/// @brief Defines static object to access dataRef and fetches its value into `var`
/// @param var Variable receiving current dataRef's value
/// @param drName Name to be used for static variable, prepended with "DR"
/// @param dataRef Name of dataRef as C string
/// @param type Type of dataRef like Int, Float, Byte
#define ASSIGN_DR_NAME(var,drName,dataRef,type)                 \
static LTDataRef DR##drName(dataRef);                           \
var = DR##drName.get##type();

/// @brief Defines static object `LTDataRef` to access dataRef and `return`s its current value
/// @param dataRef Name of dataRef as C string
/// @param type Type of dataRef like Int, Float, Byte
#define RETURN_DR(dataRef,type)                                 \
static LTDataRef DR(dataRef);                                   \
return DR.get##type();

/// Set last element of array = `0`, meant to ensure zero-termination of C strings
#define ZERO_TERM(str) str[sizeof(str)-1] = 0

namespace LTAPI {
    /// @brief Inverse for gmtime, i.e. converts `struct tm` to `time_t` in ZULU timezone
    /// @param _Tm Date/time structure to convert
    /// @return Same Date/time, converted to `time_t`in ZULU timezone
    time_t timegm(struct tm* _Tm)
    {
        time_t t = mktime(_Tm); 
        return t + (mktime(localtime(&t)) - mktime(gmtime(&t)));
    }
    
    /// @brief Fairly fast conversion to hex string.
    /// @param n The number to convert
    /// @param minChars (optional, defaults to 6) minimum number of hex digits, pre-filled with `0
    /// @return Upper-case hex string with at least `minDigits` characters
    ///
    /// Idea is taken from `std::to_chars` implementation available with C++ 17
    std::string hexStr (uint64_t n, unsigned minChars = 6)
    {
        char buf[11] = {0,0,0,0,0,0,0,0,0,0,0};
        char* last = buf + sizeof(buf)-2;       // we keep one last zero for zero-termination!
        while (last != buf)
        {
            auto c = n % 16;                    // digit to convert
            *--last = "0123456789ABCDEF"[c];    // sets the digit and decrements the pointer
            n /= 16;                            // remainder for next cycle
            if (n == 0)                         // nothing left -> done
            {
                // some more leading zeroes needed?
                if (minChars > sizeof(buf)-1)
                    minChars = sizeof(buf)-1;
                while (buf + sizeof(buf)-1 - minChars < last)
                    *--last = '0';
                return last;
            }
        }
        return "-OVFL-";                        // overflow
    }

    /// With this global variable we declare that we are setting shared dataRef information and want to ignore the resulting notification callback
    static bool gbIgnoreBecauseItsMe = false;

    /// Set the shared dataRefs for aircraft under camera
    void setCameraAcDataRefs (int tcasIdx, int modeS_id)
    {
        static LTDataRef drTcasIdx(SDR_CAMERA_TCAS_IDX);
        static LTDataRef drModeSId(SDR_CAMERA_MODES_ID);
        gbIgnoreBecauseItsMe = true;
        drTcasIdx.set(tcasIdx);
        drModeSId.set(modeS_id);
        gbIgnoreBecauseItsMe = false;
    }

}

//
// MARK: LTAPIAircraft
//
// Represents one aircraft as controlled by LiveTraffic.
//

LTAPIAircraft::LTAPIAircraft()
{}

LTAPIAircraft::~LTAPIAircraft()
{}

/// Puts together a string if at max 3 compontens:
/// 1. an identifier (flight number, call sign, key)
/// 2. a/c type (model ICAO, model human readble)
/// 3. origin/destination
/// @return Description of aircraft useful as label
std::string LTAPIAircraft::getDescription() const
{
    std::string ret;
    
    // 1. identifier
    if (info.flightNumber[0])
        ret = info.flightNumber;
    else if (info.callSign[0])
        ret = info.callSign;
    else
        ret = key;
    
    // 2. a/c type
    if (info.modelIcao[0]) {
        ret += " (";
        ret += info.modelIcao;
        ret += ')';
    }
    else if (info.model[0]) {
        ret += " (";
        ret += info.model;
        ret += ')';
    }
    
    // 3. origin/destination
    if (info.origin[0] || info.destination[0]) {
        ret += " ";
        ret += info.origin[0] ? info.origin : "?";
        ret += "-";
        ret += info.destination[0] ? info.destination : "?";
    }
    
    return ret;
}

// Main function: Updates an aircraft from LiveTraffic's dataRefs

/// Copies the provided `bulk` data and sets `bUpdated` to `true`
/// if the provided data matches this aircraft.
/// @note This function can _set_ this object's `key` for the first and only time.
bool LTAPIAircraft::updateAircraft(const LTAPIBulkData& __bulk, size_t __inSize)
{
    // first time init of this LTAPIAircraft object?
    if (key.empty()) {
        // yes, so we accept the offered aircraft as ours now:
        keyNum = (unsigned)__bulk.keyNum;
        key = LTAPI::hexStr(__bulk.keyNum);
    } else {
        // our key isn't empty, so we continue only if the aircraft offered
        // is the same!
        if (__bulk.keyNum != keyNum)
            return false;
    }
    
    // just copy the data
    bulk = __bulk;
    
    // version compatibility
    // If LiveTraffic sent v120 number of bytes then we need to fill
    // new double values from old float values as the doubles haven't been
    // transferred:
    if (__inSize < LTAPIBulkData_v122) {
        bulk.lat = bulk.lat_f;
        bulk.lon = bulk.lon_f;
        bulk.alt_ft = bulk.alt_ft_f;
    }
    
    // has been updated
    bUpdated = true;
    return true;
}

/// Copies the provided `info` data and sets `bUpdated` to `true`
/// if the provided data matches this aircraft.
/// @note This function will never overwrite `key`!
///       A new LTAPIAircraft object will always receive a call to
///       the above version (with `LTAPIBulkData`) first before receiving
///       a call to this version (with `LTAPIBulkInfoTexts`).
bool LTAPIAircraft::updateAircraft(const LTAPIBulkInfoTexts& __info, size_t __inSize)
{
    // We continue only if the aircraft offered
    // is the same as we represent!
    if (__info.keyNum != keyNum)
        return false;
    
    // just copy the data
    info = __info;

    // We don't trust nobody, so we make sure that the C strings are zero-terminated
    ZERO_TERM(info.registration);
    ZERO_TERM(info.modelIcao);
    ZERO_TERM(info.acClass);
    ZERO_TERM(info.wtc);
    ZERO_TERM(info.opIcao);
    ZERO_TERM(info.man);
    ZERO_TERM(info.model);
    ZERO_TERM(info.catDescr);
    ZERO_TERM(info.op);
    ZERO_TERM(info.callSign);
    ZERO_TERM(info.squawk);
    ZERO_TERM(info.flightNumber);
    ZERO_TERM(info.origin);
    ZERO_TERM(info.destination);
    ZERO_TERM(info.trackedBy);
    ZERO_TERM(info.cslModel);
    
    // version compatibility
    // If LiveTraffic sent v120 number of bytes then we didn't receive cslModel
    if (__inSize < LTAPIBulkInfoTexts_v122) {
        memset(info.cslModel, 0, sizeof(info.cslModel));
    }
    // If LiveTraffic sent v122 number of bytes then we receive only 24 chars of cslModel
    else if (__inSize < LTAPIBulkInfoTexts_v240) {
        memset(info.cslModel+24, 0, sizeof(info.cslModel)-24);
    }

    // has been updated
    bUpdated = true;
    return true;
}

// @brief Declare the aircraft the one under the camera (e.g. if your plugin is a camera plugin and now views this aircraft)
void LTAPIAircraft::setCameraAc ()
{
    LTAPI::setCameraAcDataRefs(getMultiIdx(), (int)keyNum);
}

/// @return Human readable string for current flight phase
std::string LTAPIAircraft::getPhaseStr () const
{
    switch (bulk.bits.phase) {
        case FPH_UNKNOWN:           return "Unknown";
        case FPH_PARKED:            return "Parked";
        case FPH_TAXI:              return "Taxi";
        case FPH_TAKE_OFF:          return "Take Off";
        case FPH_TO_ROLL:           return "Take Off Roll";
        case FPH_ROTATE:            return "Rotate";
        case FPH_LIFT_OFF:          return "Lift Off";
        case FPH_INITIAL_CLIMB:     return "Initial Climb";
        case FPH_CLIMB:             return "Climb";
        case FPH_CRUISE:            return "Cruise";
        case FPH_DESCEND:           return "Descend";
        case FPH_APPROACH:          return "Approach";
        case FPH_FINAL:             return "Final";
        case FPH_LANDING:           return "Landing";
        case FPH_FLARE:             return "Flare";
        case FPH_TOUCH_DOWN:        return "Touch Down";
        case FPH_ROLL_OUT:          return "Roll Out";
        case FPH_STOPPED_ON_RWY:    return "Stopped";
    }
    // must not get here...then we missed a value in the above switch
    return "?";
}

//
// MARK: LTAPIConnect
//

LTAPIConnect::LTAPIConnect(fCreateAcObject* _pfCreateAcObject, int numBulkAc) :
// clamp numBulkAc between 1 and 100
iBulkAc(numBulkAc < 1 ? 1 : numBulkAc > 100 ? 100 : numBulkAc),
// reserve memory for bulk data transfer from LiveTraffic
vBulkNum (new LTAPIAircraft::LTAPIBulkData[iBulkAc]),
vInfoTexts(new LTAPIAircraft::LTAPIBulkInfoTexts[iBulkAc]),
pfCreateAcObject(_pfCreateAcObject)
{
    // Create the shared dataRefs to access camera aircraft event notifications
    XPLMShareData(SDR_CAMERA_MODES_ID, xplmType_Int, nullptr, nullptr);
    XPLMShareData(SDR_CAMERA_TCAS_IDX, xplmType_Int, (XPLMDataChanged_f)(&LTAPIConnect::CameraSharedDataCB), this);
}

LTAPIConnect::~LTAPIConnect()
{
    XPLMUnshareData(SDR_CAMERA_MODES_ID, xplmType_Int, nullptr, nullptr);
    XPLMUnshareData(SDR_CAMERA_TCAS_IDX, xplmType_Int, (XPLMDataChanged_f)(&LTAPIConnect::CameraSharedDataCB), this);
}

// LiveTraffic available? (checks via XPLMFindPluginBySignature)
bool LTAPIConnect::isLTAvail ()
{
    return XPLMFindPluginBySignature(LT_PLUGIN_SIGNATURE) != XPLM_NO_PLUGIN_ID;
}

// LiveTraffic's version number
int LTAPIConnect::getLTVerNr()
{
    static LTDataRef DRVerNr("livetraffic/ver/nr");
    if (!isLTAvail())                   // LiveTraffic unavailable?
        return 0;
    if (DRVerNr.isValid())              // Can fetch version number from LT?
        return DRVerNr.getInt();
    else
        return 150;
}

/// @brief LiveTraffic's version date
/// @details Version date became available with v2.01 only. This is why 20191231 is returned in case
///          LiveTraffic is available, but not the dataRef to fetch the date from.
/// @return Version date (like 20200430 for 30-APR-2020), or constant 20191231 if unknown, or 0 if LiveTraffic is unavailable
int LTAPIConnect::getLTVerDate()
{
    static LTDataRef DRVerDate("livetraffic/ver/date");
    if (!isLTAvail())                   // LiveTraffic unavailable?
        return 0;
    if (DRVerDate.isValid())            // Can fetch version date from LT?
        return DRVerDate.getInt();
    else
        return 20191231;
}


// Does LiveTraffic display aircrafts? (Is it activated?)
bool LTAPIConnect::doesLTDisplayAc ()
{
    static LTDataRef DRAcDisplayed("livetraffic/cfg/aircrafts_displayed");
    // this is the only function which tries to find the dataRef over and over again
    if (!DRAcDisplayed.isValid())
        DRAcDisplayed.FindDataRef();
    return DRAcDisplayed.getBool();
}

// How many of them right now?
int LTAPIConnect::getLTNumAc ()
{
    RETURN_DR("livetraffic/ac/num",Int);
}

// Does it (also) control AI planes?
bool LTAPIConnect::doesLTControlAI ()
{
    RETURN_DR("livetraffic/cfg/ai_controlled",Bool);
}

// What's current simulated time in LiveTraffic (usually 'now' minus buffering period)?
time_t LTAPIConnect::getLTSimTime ()
{
    struct tm t;
    memset(&t, 0, sizeof(t));
    
    int i = 0;
    ASSIGN_DR_NAME(i, Date, "livetraffic/sim/date", Int);
    t.tm_year = i / 10000;
    i -= t.tm_year * 10000;
    t.tm_mon = i / 100 - 1;
    t.tm_mday = i % 100;

    ASSIGN_DR_NAME(i, Time, "livetraffic/sim/time", Int);
    t.tm_hour   = i / 10000;
    t.tm_min    = (i % 10000) / 100;
    t.tm_sec    = i % 100;
    
    return LTAPI::timegm(&t);
}

std::chrono::system_clock::time_point LTAPIConnect::getLTSimTimePoint ()
{
    return std::chrono::system_clock::from_time_t(getLTSimTime());
}


const MapLTAPIAircraft& LTAPIConnect::UpdateAcList (ListLTAPIAircraft* plistRemovedAc)
{
    // These are the bulk input/output dataRefs in LiveTraffic,
    // with which we fetch mass data from LiveTraffic
    static LTDataRef DRquick("livetraffic/bulk/quick");
    static LTDataRef DRexpsv("livetraffic/bulk/expensive");

    // a few sanity checks...without LT displaying aircrafts
    // and access to ac/key there is nothing to do.
    // (Calling doesLTDisplayAc before calling any other dataRef
    //  makes sure we only try accessing dataRefs when they are available.)
    const int numAc = isLTAvail() && doesLTDisplayAc() && DRquick.isValid() && DRexpsv.isValid() ? getLTNumAc() : 0;
    if (numAc <= 0) {
        // does caller want to know about removed aircrafts?
        if (plistRemovedAc)
            for (MapLTAPIAircraft::value_type& p: mapAc)
                // move all objects over to the caller's list's end
                plistRemovedAc->emplace_back(std::move(p.second));
        // clear our map
        mapAc.clear();
        return mapAc;
    }
    
    // *** There are numAc aircrafts to be reported ***
    
    // To figure out which aircraft has gone we keep an update flag
    // with the aircraft. Let's reset that flag first.
    for (MapLTAPIAircraft::value_type& p: mapAc)
        p.second->resetUpdated();
    
    // *** Read bulk info from LiveTraffic ***
    
    // Always do the rather fast call for numeric data
    int sizeLTStruct = 0;                     // not yet used, will become important once different versions exist
    if (DoBulkFetch<LTAPIAircraft::LTAPIBulkData>(numAc, DRquick, sizeLTStruct,
                                                  vBulkNum) ||
        // do the expensive call for textual data if the above one added new objects, OR
        // if 3 seconds have passed since the last call
        std::chrono::steady_clock::now() - lastExpsvFetch > sPeriodExpsv)
    {
        // expensive call for textual data
        sizeLTStruct = 0;
        DoBulkFetch<LTAPIAircraft::LTAPIBulkInfoTexts>(numAc, DRexpsv, sizeLTStruct,
                                                       vInfoTexts);
        lastExpsvFetch = std::chrono::steady_clock::now();
    }
        
    // ***  Now handle aircrafts in our map, which did _not_ get updated ***
    for (MapLTAPIAircraft::iterator iter = mapAc.begin();
         iter != mapAc.end();
         /* no loop increment*/)
    {
        // not updated?
        if (!iter->second->isUpdated()) {
            // Does caller want to take over them?
            if (plistRemovedAc)
                // here you go...your object now
                plistRemovedAc->emplace_back(std::move(iter->second));
            // in any case: remove from our map and increment to next element
            iter = mapAc.erase(iter);
        }
        else
            // go to next element (without removing this one)
            iter++;
    }
    
    // We're done, return the result
    return mapAc;
}

// Finds an aircraft for a given multiplayer slot
SPtrLTAPIAircraft LTAPIConnect::getAcByMultIdx (int multiIdx) const
{
    // sanity check: Don't search for 0...there are too many of them
    if (multiIdx < 1)
        return SPtrLTAPIAircraft();
    
    // search the map for a matching aircraft
    MapLTAPIAircraft::const_iterator iter =
    std::find_if(mapAc.cbegin(), mapAc.cend(),
                 [multiIdx](const MapLTAPIAircraft::value_type& pair)
                 { return pair.second->getMultiIdx() == multiIdx; });
    
    // return a copy of the pointer if found
    return iter == mapAc.cend() ? SPtrLTAPIAircraft() : iter->second;
}


// Returns the aircraft being viewed in LiveTraffic's camera view, if any
SPtrLTAPIAircraft LTAPIConnect::getAcInCameraView() const
{
    // search the map for a matching aircraft
    MapLTAPIAircraft::const_iterator iter =
        std::find_if(mapAc.cbegin(), mapAc.cend(),
            [](const MapLTAPIAircraft::value_type& pair)
            { return pair.second->isOnCamera(); });

    // return a copy of the pointer if found
    return iter == mapAc.cend() ? SPtrLTAPIAircraft() : iter->second;
}


// LTAPIConnect::Clear camera information, ie. delcare that no aircraft is currently being viewed
void clearCameraInfo ()
{
    LTAPI::setCameraAcDataRefs(0, 0);
}


// fetch bulk data and create/update aircraft objects
template <class T>
bool LTAPIConnect::DoBulkFetch (int numAc, LTDataRef& DR, int& outSizeLT,
                                std::unique_ptr<T[]> &vBulk)
{
    // later return value: Did we add any new objects?
    bool ret = false;
    
    // Size negotiation first (we need to do that before _every_ call
    // because in theory there could be another plugin using a different
    // version of LTAPI doing calls before or after us).
    // Array element size will always be set by LTAPI.
    // The return value (size filled by LT) will become important once
    // size _can_ differ at all and we need to cater for LTAPI being
    // bigger than what LT fills. Not yet possible as this is the
    // initial version on both sides. So we don't yet use the result.
    outSizeLT = DR.getData(NULL, 0, sizeof(T));
    
    // outer loop: get bulk data (iBulkAc number of a/c per request) from LT
    for (int ac = 0;
         ac < numAc;
         ac += iBulkAc)
    {
        // get a bulk of data from LiveTraffic
        // (std::min(...iBulkAc) makes sure we don't exceed our array)
        const int acRcvd = std::min (DR.getData(vBulk.get(),
                                                ac * sizeof(T),
                                                iBulkAc * sizeof(T)) / int(sizeof(T)),
                                     iBulkAc);
        
        // inner loop: copy the received data into the aircraft objects
        for (int i = 0; i < acRcvd; i++)
        {
            const T& bulk = vBulk[i];
            
            // try to find the matching aircraft object in out map
            const std::string key = LTAPI::hexStr(bulk.keyNum);
            MapLTAPIAircraft::iterator iter = mapAc.find(key);
            if (iter == mapAc.end())            // didn't find, need new one
            {
                // create a new aircraft object
                assert(pfCreateAcObject);
                iter = mapAc.emplace(key, pfCreateAcObject()).first;
                // tell caller we added new objects
                ret = true;
            }
            
            // copy the bulk data
            assert(iter != mapAc.end());
            iter->second->updateAircraft(bulk, outSizeLT);
        } // inner loop processing received bulk data
    } // outer loop fetching bulk data from LT
    
    return ret;
}


// shared DataRef event notification
void LTAPIConnect::CameraSharedDataCB (LTAPIConnect* me)
{
    // Ignore the callback when data is changed by us
    if (LTAPI::gbIgnoreBecauseItsMe)
        return;
    
    // Fetch the aircraft id from LiveTraffic
    int modeS_id = 0;
    SPtrLTAPIAircraft spCamAc;
    ASSIGN_DR_NAME(modeS_id, id, SDR_CAMERA_MODES_ID, Int);
    
    // search the map for a matching aircraft that is _now_ under the camera
    if (modeS_id) {
        char keyHex[10];
        snprintf ( keyHex, sizeof(keyHex), "%06X", (unsigned int)modeS_id );
        MapLTAPIAircraft::iterator iter = me->mapAc.find(keyHex);
        if (iter != me->mapAc.end())
            spCamAc = iter->second;
    }

    // our data still holds the aircraft that was _previously_ under the camera
    SPtrLTAPIAircraft spPrevCamAc = me->getAcInCameraView();
    
    // Inform the aircraft
    if (spCamAc)                    // there is a (new) aircraft under the camera
        spCamAc->toggleCamera(true, spPrevCamAc);
    else if (spPrevCamAc)           // there is none now, but maybe there was one before?
        spPrevCamAc->toggleCamera(false, SPtrLTAPIAircraft());
}

//
// MARK: LTDataRef
//

LTDataRef::LTDataRef (std::string _sDataRef) :
sDataRef(_sDataRef)
{}

// Found the dataRef and it contains formats we can work with?
/// @note Not const! Will call FindDataRef() to try becoming valid.
bool LTDataRef::isValid ()
{
    if (needsInit()) FindDataRef();
    return bValid;
}

// binds to the dataRef and sets bValid
bool LTDataRef::FindDataRef ()
{
    dataRef = XPLMFindDataRef(sDataRef.c_str());
    // check available data types; we only work with a subset
    dataTypes = dataRef ? (XPLMGetDataRefTypes(dataRef) & usefulTypes) : xplmType_Unknown;
    return bValid = dataTypes != xplmType_Unknown;
}

int LTDataRef::getInt()
{
    if (needsInit()) FindDataRef();
    return XPLMGetDatai(dataRef);
}

float LTDataRef::getFloat()
{
    if (needsInit()) FindDataRef();
    return XPLMGetDataf(dataRef);
}

int LTDataRef::getData(void* pOut, int inOffset, int inMaxBytes)
{
    if (needsInit()) FindDataRef();
    return XPLMGetDatab(dataRef, pOut, inOffset, inMaxBytes);
}

void LTDataRef::set(int i)
{
    if (needsInit()) FindDataRef();
    XPLMSetDatai(dataRef, i);
}

void LTDataRef::set(float f)
{
    if (needsInit()) FindDataRef();
    XPLMSetDataf(dataRef, f);
}

