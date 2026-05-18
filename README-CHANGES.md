# Local changes summary (this fork / branch)

This document describes edits made in this workspace that differ from the usual upstream behavior, plus how to build on Windows.

---

## 1. Dock as soon as jetways are ready (automatic *and* manual mode)

**Problem:** openSAM could reach `CAN_DOCK` (jetways selected and configured) but **still wait** for something else to request a dock—typically the DGS flow calling `RequestDock()`, dock/toggle commands, or the automatic-mode-only check used in an earlier tweak.

**Change:** In `CAN_DOCK`, with **beacon off**, the plugin **always** starts docking (automatic or manual jetway selection). No separate dock request needed.

**Code (`plane.cpp`):** after handling beacon-on (`new_state = IDLE` with **`break`** so docking is not started in the same frame), a single block runs `SetupDockUndock` and sets `DOCKING`.

**Effect:**

- User aircraft: as soon as the state machine reaches `CAN_DOCK` and the beacon guard is satisfied, it transitions to `DOCKING` and runs the normal staggered `SetupDockUndock` sequence—whether jetways were picked automatically or manually in the UI.
- **Undock behavior is unchanged:** beacon ON (or undock/toggle request) while `DOCKED` still drives undocking as before.
- **Beacon fix:** if the beacon turns on while in `CAN_DOCK`, we go to `IDLE` and **break** immediately so we never start `DOCKING` in the same evaluation (the old `auto_mode()` path could theoretically overwrite `IDLE` with `DOCKING`).

**Manual UI fix (`os_ui.cpp`):** In manual mode, `active_jws_` used to be filled only when pressing **Dock**, so choosing jetway checkboxes alone never advanced the state machine. **`SyncManualSelectionFromJwButtons`** runs after each jetway checkbox change (in `SELECT_JWS`) and rebuilds `active_jws_` from the widgets, so docking can start as soon as a valid selection exists—**Dock** is still available if you want to confirm with a click.

---

## 2. Multiplayer jetways (xPilot / Traffic Global / LiveTraffic)

During experimentation, docking was briefly gated on a “beacon seen ON at least once” flag so MP traffic would not auto-dock on load. That was **reverted** per your request: parked traffic with beacon off should **dock on load** again.

**Current behavior:** `MpPlane_*::dock_requested()` still returns **`true`** in:

- `mpadapter_xpilot.cpp`
- `mpadapter_tgxp.cpp`
- `mpadapter_lt.cpp`

So MP planes keep the original “always request dock when the state machine asks” semantics, which pairs with `plane.cpp`’s `CAN_DOCK` logic for immediate dock when applicable.

---

## 3. Version string

**File:** `version.mak`

- Set to **`VERSION=v4.5.5`** (bump from your previous **v4.5.4**).

`Makefile.common` generates `version.h` from this at build time (`VERSION` / `VERSION_SHORT` for the plugin UI and startup log).

---

## 4. Windows build (MinGW)

**Prerequisites:**

- MinGW-w64 with `g++` and `mingw32-make` on `PATH`.
- **X-Plane SDK** checked out or copied so it sits **next to** the `openSAM` repo folder, not inside it:
  - If the repo is `...\GitHub\openSAM`, the SDK must be `...\GitHub\SDK\`
  - Required path example: `...\GitHub\SDK\CHeaders\XPLM\XPLMDataAccess.h`
- **`xplib`** sibling folder: `...\GitHub\xplib\` (already expected by `Makefile.common`).
- For XP11 builds / `make all`: **`libOpenAL32`** at `...\GitHub\libOpenAL32\` (see `Makefile.mgw64`).

**Example (XP12 plugin only):**

```bat
cd /d C:\path\to\openSAM
set PATH=C:\ProgramData\mingw64\mingw64\bin;%PATH%
mingw32-make -f Makefile.mgw64 install_xp12
```

Output is copied to:

`openSAM-pkg\openSAM\win_x64\openSAM.xpl`

If the build fails with **missing `XPLM*.h`**, the SDK is not at `..\SDK` relative to the `openSAM` directory.

---

## 5. Files touched in these changes

| File | Change |
|------|--------|
| `plane.cpp` | `CAN_DOCK`: dock immediately when beacon off (auto + manual); `break` after beacon→IDLE |
| `version.mak` | `VERSION=v4.5.5` |
| `mpadapter_*.cpp` | Confirmed MP `dock_requested()` returns `true` (revert of experimental gating) |
| `os_ui.cpp` | Sync `active_jws_` from jetway checkboxes on each change in manual `SELECT_JWS` |
| `README-CHANGES.md` | This file |

---

## 6. Relation to AutoGate

AutoGate’s DGS flow includes special handling for “parked at gate on load” (lead-in / engage without jumping straight to a fully docked state). The change here is **only** in openSAM’s jetway state machine: **automatic user-aircraft dock** no longer depends on DGS reaching `DONE` or a separate `RequestDock()` if you already use automatic jetway selection.

For questions about upstream openSAM, see the main [README.md](README.md).
