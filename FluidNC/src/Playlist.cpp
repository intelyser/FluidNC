// Playlist module: runs a user-ordered list of G-code files from the local
// filesystem, one after the other, optionally repeating.  Designed to be driven
// by a very simple web page (data/ui/kiosk/index.html) through a handful of
// "$Playlist/..." commands sent over the existing /command?plain= HTTP route.
//
// Everything runs on the polling task (commands and Module::poll), so the
// state below needs no locking.  The only things that must happen on the
// protocol task -- starting a job, feed hold, reset, homing -- are done by
// queueing events, exactly like a pin-triggered macro does.
//
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Module.h"
#include "Settings.h"
#include "Protocol.h"
#include "System.h"
#include "State.h"
#include "Job.h"
#include "Macro.h"
#include "Logging.h"
#include "FluidPath.h"
#include "Machine/Axes.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace {
    // ---------------------------------------------------------------- types

    struct Entry {
        std::string name;
        bool        checked = false;
        uintmax_t   size    = 0;
    };

    enum class PState { Idle, Launching, Running, Between, Ending, GoingHome };

    // -------------------------------------------------------------- globals

    const char* const CFG_FILE = "/playlist.cfg";
    const char* const GCODE_DIR = "/gcode";  // user G-code files live here, not in the LocalFS root

    Macro    s_macro { "Playlist" };  // runs "$LocalFS/Run=..." or the go-home move
    volatile bool s_launch_done = false;
    volatile bool s_launch_ok   = false;

    PState                   s_state = PState::Idle;
    std::vector<std::string> s_seq;  // names captured at Start
    size_t                   s_idx = 0;
    bool                     s_repeat = false;
    bool                     s_held   = false;  // user pressed Pause
    bool                     s_resume_pending = false;
    std::string              s_message;  // shown on the page when idle ("Finished", "Error ...")
    uint32_t                 s_abort0 = 0;
    int                      s_end_stage = 0;
    TickType_t               s_stage_time = 0;

    TickType_t now() {
        return xTaskGetTickCount();
    }
    bool older_than(TickType_t since, uint32_t ms) {
        return (now() - since) > pdMS_TO_TICKS(ms);
    }

    // ----------------------------------------------------------- launch event

    // Runs on the protocol task.  Refuses to start if a job is already active,
    // the same rule MacroEvent uses.
    class LaunchEvent : public Event {
    public:
        void run(void*) const override {
            s_launch_ok = false;
            if (!Job::active()) {
                s_launch_ok = s_macro.run(nullptr);
            }
            s_launch_done = true;
        }
    };
    const LaunchEvent s_launch_event {};

    void launch_macro(const std::string& gcode) {
        s_macro.set(gcode);
        s_launch_done = false;
        s_launch_ok   = false;
        protocol_send_event(&s_launch_event);
    }

    // --------------------------------------------------------- list storage

    bool is_gcode_name(const std::string& n) {
        auto dot = n.rfind('.');
        if (dot == std::string::npos) {
            return false;
        }
        std::string ext = n.substr(dot + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return (char)tolower(c); });
        return ext == "gcode" || ext == "gc" || ext == "nc" || ext == "ngc";
    }

    std::string cfg_path() {
        return FluidPath { CFG_FILE, LocalFS }.string();
    }

    // Saved order, as "flag name" pairs, plus the repeat flag.
    void load_cfg(std::vector<Entry>& list, bool& repeat) {
        list.clear();
        repeat = false;
        FILE* f = fopen(cfg_path().c_str(), "r");
        if (!f) {
            return;
        }
        char line[160];
        while (fgets(line, sizeof line, f)) {
            std::string s(line);
            while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) {
                s.pop_back();
            }
            if (s.rfind("repeat=", 0) == 0) {
                repeat = s.size() > 7 && s[7] == '1';
            } else if (s.size() > 2 && s[1] == '|') {
                Entry e;
                e.checked = s[0] == '1';
                e.name    = s.substr(2);
                list.push_back(e);
            }
        }
        fclose(f);
    }

    void save_cfg(const std::vector<Entry>& list, bool repeat) {
        FILE* f = fopen(cfg_path().c_str(), "w");
        if (!f) {
            log_error("Playlist: cannot write " << CFG_FILE);
            return;
        }
        fprintf(f, "repeat=%d\n", repeat ? 1 : 0);
        for (auto& e : list) {
            fprintf(f, "%c|%s\n", e.checked ? '1' : '0', e.name.c_str());
        }
        fclose(f);
    }

    // G-code files currently in the LocalFS root, with sizes.
    std::vector<Entry> scan_files() {
        std::vector<Entry> found;
        try {
            FluidPath dir { GCODE_DIR, LocalFS };
            std::error_code ec;
            stdfs::create_directories(dir, ec);  // first boot: folder does not exist yet
            for (auto const& de : stdfs::directory_iterator { dir }) {
                if (de.is_directory()) {
                    continue;
                }
                std::string n = de.path().filename().string();
                if (is_gcode_name(n)) {
                    Entry e;
                    e.name = n;
                    e.size = de.file_size();
                    found.push_back(e);
                }
            }
        } catch (std::exception const& ex) {
            log_error("Playlist: cannot list files: " << ex.what());
        }
        return found;
    }

    // Saved order, minus files that no longer exist, plus new files (unchecked,
    // alphabetical) at the end.
    std::vector<Entry> merged_list(bool& repeat) {
        std::vector<Entry> saved;
        load_cfg(saved, repeat);
        auto found = scan_files();
        std::sort(found.begin(), found.end(), [](const Entry& a, const Entry& b) { return a.name < b.name; });

        std::vector<Entry> out;
        for (auto& s : saved) {
            for (auto& f : found) {
                if (f.name == s.name) {
                    Entry e = s;
                    e.size  = f.size;
                    out.push_back(e);
                    break;
                }
            }
        }
        for (auto& f : found) {
            bool known = false;
            for (auto& o : out) {
                if (o.name == f.name) {
                    known = true;
                    break;
                }
            }
            if (!known) {
                out.push_back(f);
            }
        }
        return out;
    }

    // ---------------------------------------------------------------- status

    std::string json_escape(const std::string& s) {
        std::string o;
        for (char c : s) {
            if (c == '"' || c == '\\') {
                o += '\\';
            }
            if ((unsigned char)c >= 0x20) {
                o += c;
            }
        }
        return o;
    }

    const char* state_word() {
        switch (s_state) {
            case PState::Idle:
                return "idle";
            case PState::Ending:
            case PState::GoingHome:
                return "ending";
            default:
                return s_held ? "paused" : "running";
        }
    }

    std::string status_json() {
        bool  repeat;
        auto  list = merged_list(repeat);
        std::string j = "{\"state\":\"";
        j += state_word();
        j += "\",\"message\":\"" + json_escape(s_message) + "\"";
        j += ",\"alarm\":";
        j += (state_is(State::Alarm) || state_is(State::ConfigAlarm)) ? "true" : "false";
        j += ",\"repeat\":";
        j += (s_state == PState::Idle ? repeat : s_repeat) ? "true" : "false";
        j += ",\"current\":\"";
        if (s_state != PState::Idle && s_state != PState::Ending && s_state != PState::GoingHome && s_idx < s_seq.size()) {
            j += json_escape(s_seq[s_idx]);
        }
        j += "\",\"position\":" + std::to_string(s_idx + 1) + ",\"total\":" + std::to_string(s_seq.size());
        j += ",\"files\":[";
        for (size_t i = 0; i < list.size(); i++) {
            if (i) {
                j += ",";
            }
            j += "{\"n\":\"" + json_escape(list[i].name) + "\",\"c\":" + (list[i].checked ? "1" : "0") + ",\"s\":" +
                 std::to_string((unsigned long)list[i].size) + "}";
        }
        j += "]}";
        return j;
    }

    // -------------------------------------------------------------- helpers

    bool machine_alarm() {
        return state_is(State::Alarm) || state_is(State::ConfigAlarm) || state_is(State::Critical);
    }

    void go_idle(const char* message) {
        s_state          = PState::Idle;
        s_held           = false;
        s_resume_pending = false;
        s_message        = message;
    }

    void launch_current() {
        s_abort0 = Job::abort_count();
        launch_macro(std::string("$LocalFS/Run=") + GCODE_DIR + "/" + s_seq[s_idx]);
        s_state = PState::Launching;
    }

    // ------------------------------------------------------------- commands

    // $Playlist/Status
    Error cmd_status(const char*, AuthenticationLevel, Channel& out) {
        log_stream(out, "[PLAYLIST:" << status_json());
        return Error::Ok;
    }

    // $Playlist/Save=<repeat 0|1>|<flag><name>|<flag><name>...   flag is 1 (checked) or 0
    Error cmd_save(const char* param, AuthenticationLevel, Channel& out) {
        std::string        s(param ? param : "");
        std::vector<Entry> list;
        bool               repeat = false;
        size_t             pos    = 0;
        bool               first  = true;
        while (pos <= s.size()) {
            size_t      end = s.find('|', pos);
            std::string tok = s.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
            if (first) {
                repeat = !tok.empty() && tok[0] == '1';
                first  = false;
            } else if (tok.size() > 1) {
                Entry e;
                e.checked = tok[0] == '1';
                e.name    = tok.substr(1);
                if (is_gcode_name(e.name)) {
                    list.push_back(e);
                }
            }
            if (end == std::string::npos) {
                break;
            }
            pos = end + 1;
        }
        save_cfg(list, repeat);
        if (s_state != PState::Idle) {
            s_repeat = repeat;  // repeat can be toggled while running
        }
        return Error::Ok;
    }

    // $Playlist/Start
    Error cmd_start(const char*, AuthenticationLevel, Channel& out) {
        if (s_state != PState::Idle) {
            return Error::Ok;  // already running; ignore double clicks
        }
        if (machine_alarm()) {
            s_message = "The machine is locked. Press Unlock first.";
            return Error::Ok;
        }
        if (!state_is(State::Idle)) {
            s_message = "The machine is busy. Try again in a moment.";
            return Error::Ok;
        }
        bool repeat;
        auto list = merged_list(repeat);
        s_seq.clear();
        for (auto& e : list) {
            if (e.checked) {
                s_seq.push_back(e.name);
            }
        }
        if (s_seq.empty()) {
            s_message = "Tick at least one file first.";
            return Error::Ok;
        }
        s_repeat  = repeat;
        s_idx     = 0;
        s_held    = false;
        s_message = "";
        launch_current();
        return Error::Ok;
    }

    // $Playlist/Pause
    Error cmd_pause(const char*, AuthenticationLevel, Channel& out) {
        if (s_state == PState::Idle || s_state == PState::Ending || s_state == PState::GoingHome) {
            return Error::Ok;
        }
        s_held = true;
        log_info("Playlist: pause requested, machine state " << (int)sys.state());
        if (state_is(State::Cycle)) {
            protocol_send_event(&feedHoldEvent);
        }
        return Error::Ok;
    }

    // $Playlist/Resume
    Error cmd_resume(const char*, AuthenticationLevel, Channel& out) {
        if (!s_held) {
            return Error::Ok;
        }
        s_held = false;
        if (state_is(State::Hold)) {
            s_resume_pending = true;  // sent from poll() once the hold has finished decelerating
        }
        return Error::Ok;
    }

    // $Playlist/End : stop now, then go to the home position
    Error cmd_end(const char*, AuthenticationLevel, Channel& out) {
        if (s_state == PState::Idle) {
            // Nothing running from the playlist, but still honour the button:
            // stop whatever the machine is doing and go home.
            s_seq.clear();
        }
        if (s_state == PState::Ending || s_state == PState::GoingHome) {
            return Error::Ok;
        }
        s_state          = PState::Ending;
        s_end_stage      = 0;
        s_held           = false;
        s_resume_pending = false;
        s_message        = "";
        s_stage_time     = now();
        return Error::Ok;
    }

    // ------------------------------------------------------------ the module

    class PlaylistModule : public Module {
    public:
        explicit PlaylistModule(const char* name) : Module(name) {}

        void init() override {
            scan_files();  // creates the /gcode folder if missing
            new ReportCommand("PLQ", "Playlist/Status", cmd_status, anyState, WG);
            new ReportCommand("PLS", "Playlist/Save", cmd_save, anyState, WG);
            new ReportCommand("PLG", "Playlist/Start", cmd_start, anyState, WG);
            new ReportCommand("PLP", "Playlist/Pause", cmd_pause, anyState, WG);
            new ReportCommand("PLR", "Playlist/Resume", cmd_resume, anyState, WG);
            new ReportCommand("PLE", "Playlist/End", cmd_end, anyState, WG);
            log_info("Playlist ready");
        }

        void poll() override {
            // Resume after a user pause, once the hold has finished
            if (s_resume_pending) {
                if (state_is(State::Hold)) {
                    if (sys.suspend().bit.holdComplete) {
                        protocol_send_event(&cycleStartEvent);
                        s_resume_pending = false;
                    }
                } else {
                    s_resume_pending = false;
                }
            }

            switch (s_state) {
                case PState::Idle:
                    return;

                case PState::Launching:
                    if (!s_launch_done) {
                        return;
                    }
                    if (!s_launch_ok) {
                        go_idle("Could not start the file. Is the machine busy?");
                        return;
                    }
                    s_state = PState::Running;
                    return;

                case PState::Running:
                    if (machine_alarm()) {
                        go_idle("The machine stopped (alarm). Press Unlock, then Start.");
                        return;
                    }
                    if (Job::active()) {
                        return;
                    }
                    if (Job::abort_count() != s_abort0) {
                        go_idle("Stopped because of an error in the file.");
                        return;
                    }
                    // File finished; wait for the last move to complete
                    if (!state_is(State::Idle)) {
                        return;
                    }
                    s_idx++;
                    if (s_idx >= s_seq.size()) {
                        if (s_repeat) {
                            s_idx = 0;
                        } else {
                            go_idle("Finished.");
                            return;
                        }
                    }
                    s_state = PState::Between;
                    return;

                case PState::Between:
                    if (machine_alarm()) {
                        go_idle("The machine stopped (alarm). Press Unlock, then Start.");
                        return;
                    }
                    if (s_held || !state_is(State::Idle)) {
                        return;
                    }
                    launch_current();
                    return;

                case PState::Ending:
                    poll_end();
                    return;

                case PState::GoingHome:
                    // Give the homing / move event time to be picked up
                    if (!older_than(s_stage_time, 500)) {
                        return;
                    }
                    if (machine_alarm()) {
                        go_idle("Could not reach the home position (alarm).");
                        return;
                    }
                    if (s_launch_done && !Job::active() && state_is(State::Idle)) {
                        go_idle("Stopped. Machine is at home.");
                    } else if (older_than(s_stage_time, 120000)) {
                        go_idle("Going home took too long.");
                    }
                    return;
            }
        }

    private:
        void poll_end() {
            switch (s_end_stage) {
                case 0:  // bring the motion to a controlled stop
                    if (state_is(State::Cycle)) {
                        protocol_send_event(&feedHoldEvent);
                        s_end_stage = 1;
                    } else if (state_is(State::Hold)) {
                        s_end_stage = 1;
                    } else {
                        s_end_stage = 2;  // nothing moving
                    }
                    s_stage_time = now();
                    return;
                case 1:  // wait for the hold to complete, so position is kept
                    if ((state_is(State::Hold) && sys.suspend().bit.holdComplete) || older_than(s_stage_time, 15000) ||
                        state_is(State::Idle)) {
                        s_end_stage = 2;
                        s_stage_time = now();
                    }
                    return;
                case 2:  // reset: drops the running file(s); after a completed hold this keeps position
                    if (Job::active() || state_is(State::Hold)) {
                        protocol_send_event(&rtResetEvent);
                    }
                    s_end_stage  = 3;
                    s_stage_time = now();
                    return;
                case 3:  // wait for the machine to settle
                    if (!older_than(s_stage_time, 300)) {
                        return;
                    }
                    if (machine_alarm()) {
                        go_idle("Stopped, but the machine raised an alarm. Press Unlock.");
                        return;
                    }
                    if (!Job::active() && state_is(State::Idle)) {
                        s_end_stage = 4;
                    } else if (older_than(s_stage_time, 8000)) {
                        go_idle("Could not stop cleanly. Press Unlock.");
                    }
                    return;
                case 4:  // go to the home position: machine zero (no homing cycle, so no limit switches are needed)
                    s_state      = PState::GoingHome;
                    s_stage_time = now();
                    launch_macro("G90&G53 G0 X0 Y0");
                    return;
            }
        }
    };

    ModuleFactory::InstanceBuilder<PlaylistModule> __attribute__((init_priority(112))) playlist_module("playlist", true);
}  // namespace
