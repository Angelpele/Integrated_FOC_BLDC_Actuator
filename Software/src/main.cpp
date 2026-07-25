//===================================================================================================//
//                                            Motor Monitor                                          //
//                                   (C++ / Dear ImGui / ImPlot port)                                //
//===================================================================================================//
// Native rewrite of motor_monitor.py. Immediate-mode GUI (Dear ImGui) + GPU-accelerated
// real-time plotting (ImPlot) + a lightweight cross-platform serial backend (serial_port.hpp).
// No Qt, no Python interpreter, no per-widget object overhead: the whole UI is redrawn from
// plain data every frame, so it scales to very high sample/refresh rates with near-zero CPU cost.

#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "serial_port.hpp"
#include "ring_series.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// =====================================================================================
//  DATA MODEL
// =====================================================================================

static constexpr int PLOT_LEN = 600;

struct Telemetry {
    int   id = 0;
    float pvdd = 0, iq = 0, id_ = 0, i3 = 0;
    float a = 0, b = 0, c = 0;
    int   nfault = 0, noctw = 0;
    float ntc = 0;
    float m_ang = 0, red_ang = 0;
    float m_rpm = 0, red_rpm = 0;
};

// Parses one CSV telemetry line (15 fields), matching the ESP32 firmware's
// telemetryLoop() snprintf format exactly. Returns false if malformed.
static bool parseTelemetry(const std::string& line, Telemetry& out) {
    std::vector<std::string> parts;
    parts.reserve(15);
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) parts.push_back(tok);
    if (parts.size() != 15) return false;

    try {
        out.id      = std::stoi(parts[0]);
        out.pvdd    = std::stof(parts[1]);
        out.iq      = std::stof(parts[2]);
        out.id_     = std::stof(parts[3]);
        out.i3      = std::stof(parts[4]);
        out.a       = std::stof(parts[5]);
        out.b       = std::stof(parts[6]);
        out.c       = std::stof(parts[7]);
        out.nfault  = std::stoi(parts[8]);
        out.noctw   = std::stoi(parts[9]);
        out.ntc     = std::stof(parts[10]);
        out.m_ang   = std::stof(parts[11]);
        out.red_ang = std::stof(parts[12]);
        out.m_rpm   = std::stof(parts[13]);
        out.red_rpm = std::stof(parts[14]);
    } catch (...) {
        return false;
    }
    return true;
}

// =====================================================================================
//  SERIAL WORKER (background thread)
// =====================================================================================

class SerialWorker {
public:
    void start(const std::string& device, int baud) {
        stop();
        stopFlag_ = false;
        fatal_.clear();
        thread_ = std::thread(&SerialWorker::run, this, device, baud);
    }

    void stop() {
        stopFlag_ = true;
        if (thread_.joinable()) thread_.join();
        port_.close();
    }

    bool running() const { return thread_.joinable() && !stopFlag_; }

    // Drains everything collected since the last call. Thread-safe, cheap (mutex + swap).
    void drain(std::vector<Telemetry>& outFrames, std::vector<std::string>& outLines,
               std::vector<std::string>& outErrors, std::string& outFatal) {
        std::lock_guard<std::mutex> lk(mtx_);
        outFrames.swap(frames_);
        outLines.swap(lines_);
        outErrors.swap(errors_);
        frames_.clear();
        lines_.clear();
        errors_.clear();
        outFatal = fatal_;
        fatal_.clear();
    }

    bool writeLine(const std::string& s) {
        return port_.isOpen() && port_.write(s);
    }

private:
    void run(std::string device, int baud) {
        std::string err;
        if (!port_.open(device, baud, err)) {
            std::lock_guard<std::mutex> lk(mtx_);
            fatal_ = "Cannot open " + device + ": " + err;
            return;
        }

        std::string buf;
        uint8_t chunk[512];
        int consecutiveErrors = 0;

        while (!stopFlag_) {
            int n = port_.readSome(chunk, sizeof(chunk));
            if (n < 0) {
                std::lock_guard<std::mutex> lk(mtx_);
                fatal_ = "Port lost (read error)";
                break;
            }
            if (n == 0) continue; // timeout, poll stop flag again
            consecutiveErrors = 0;
            buf.append(reinterpret_cast<char*>(chunk), n);

            size_t pos;
            while ((pos = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, pos);
                buf.erase(0, pos + 1);
                // trim CR / whitespace
                while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
                    line.pop_back();
                if (line.empty()) continue;

                Telemetry t;
                bool ok = parseTelemetry(line, t);

                std::lock_guard<std::mutex> lk(mtx_);
                lines_.push_back(line);
                if (ok) {
                    frames_.push_back(t);
                } else if (line.find(',') != std::string::npos) {
                    // Only flag as parse error if it looked like a data line.
                    errors_.push_back("Parse error: " + line.substr(0, 60));
                    if (++consecutiveErrors >= 20) {
                        fatal_ = "Too many parse errors";
                        break;
                    }
                }
            }
        }
        port_.close();
    }

    SerialPort port_;
    std::thread thread_;
    std::atomic<bool> stopFlag_{true};

    std::mutex mtx_;
    std::vector<Telemetry>  frames_;
    std::vector<std::string> lines_;
    std::vector<std::string> errors_;
    std::string fatal_;
};

// =====================================================================================
//  PID ROW WIDGET  (label + Kp/Ki/Kd sliders + order-of-magnitude multiplier + send)
// =====================================================================================

struct PIDParam {
    int   sliderVal = 0;     // 0..1000
    int   multIdx   = 3;     // index into kMultipliers, default x1
};

struct PIDRow {
    std::string name;    // e.g. "Position Loop"
    std::string prefix;  // e.g. "POS"  -> CMD:PID_POS_KP:...
    ImVec4 color;
    PIDParam kp, ki, kd;
};

static const char* kMultiplierLabels[] = {
    "x1000", "x100", "x10", "x1", "x0.1", "x0.01", "x0.001", "x0.0001"
};
static const float kMultiplierValues[] = {
    1000.0f, 100.0f, 10.0f, 1.0f, 0.1f, 0.01f, 0.001f, 0.0001f
};

static float pidParamValue(const PIDParam& p) {
    return static_cast<float>(p.sliderVal) * kMultiplierValues[p.multIdx];
}

// Renders one PID loop's 3 sliders; returns a command string when "Send" is clicked, else "".
static std::string drawPIDRow(PIDRow& row) {
    std::string sendCmd;
    ImGui::PushID(row.prefix.c_str());

    ImGui::TextColored(row.color, "  %s", row.name.c_str());
    ImGui::Separator();

    PIDParam* params[3] = { &row.kp, &row.ki, &row.kd };
    const char* labels[3] = { "Kp", "Ki", "Kd" };

    for (int i = 0; i < 3; i++) {
        ImGui::PushID(i);
        ImGui::SetNextItemWidth(28);
        ImGui::TextUnformatted(labels[i]);
        ImGui::SameLine(50);

        ImGui::SetNextItemWidth(160);
        ImGui::SliderInt("##slider", &params[i]->sliderVal, 0, 1000);
        ImGui::SameLine();

        ImGui::SetNextItemWidth(80);
        ImGui::Combo("##mult", &params[i]->multIdx, kMultiplierLabels, 8);
        ImGui::SameLine();

        float val = pidParamValue(*params[i]);
        ImGui::TextColored(row.color, "%9.4f", val);
        ImGui::PopID();
    }

    if (ImGui::Button("Send")) {
        char cmd[128];
        std::snprintf(cmd, sizeof(cmd), "CMD:PID_%s_KP:%.4f_KI:%.4f_KD:%.4f\n",
                      row.prefix.c_str(), pidParamValue(row.kp),
                      pidParamValue(row.ki), pidParamValue(row.kd));
        sendCmd = cmd;
    }
    ImGui::Spacing();
    ImGui::PopID();
    return sendCmd;
}

// =====================================================================================
//  METRIC CARD  (dashboard tile with warn/danger color thresholds)
// =====================================================================================

struct MetricCard {
    const char* name;
    const char* unit;
    float value = 0.0f;
    bool  hasWarn = false, hasDanger = false;
    float warn = 0.0f, danger = 0.0f;
    bool  highIsBad = true;
};

static void drawMetricCard(const MetricCard& m, const char* fmt) {
    ImVec4 color = ImVec4(0.0f, 0.83f, 0.67f, 1.0f); // teal (#00d4aa)
    if (m.hasDanger && (m.highIsBad ? m.value >= m.danger : m.value <= m.danger))
        color = ImVec4(1.0f, 0.30f, 0.43f, 1.0f); // red
    else if (m.hasWarn && (m.highIsBad ? m.value >= m.warn : m.value <= m.warn))
        color = ImVec4(1.0f, 0.70f, 0.28f, 1.0f); // amber

    ImGui::BeginGroup();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.086f, 0.106f, 0.153f, 1.0f));
    ImGui::BeginChild(m.name, ImVec2(190, 78), true);
    ImGui::TextColored(ImVec4(0.48f, 0.51f, 0.60f, 1.0f), "%s", m.name);
    ImGui::TextColored(color, fmt, m.value);
    ImGui::TextColored(ImVec4(0.35f, 0.38f, 0.44f, 1.0f), "%s", m.unit);
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::EndGroup();
}

// =====================================================================================
//  APP STATE
// =====================================================================================

struct RingSet {
    RingSeries pvdd{PLOT_LEN}, iq{PLOT_LEN}, id_{PLOT_LEN}, i3{PLOT_LEN};
    RingSeries m_rpm{PLOT_LEN}, red_rpm{PLOT_LEN}, ntc{PLOT_LEN};
    RingSeries m_ang{PLOT_LEN}, red_ang{PLOT_LEN};
    RingSeries a{PLOT_LEN}, b{PLOT_LEN}, c{PLOT_LEN};
    RingSeries sp_pos{PLOT_LEN}, sp_vel{PLOT_LEN}, sp_torque{PLOT_LEN};
};

struct AppState {
    SerialWorker worker;
    bool connected = false;

    std::vector<PortInfo> ports;
    int  portIdx = 0;
    int  baudIdx = 4; // 115200
    static constexpr const char* kBauds[] =
        {"9600","19200","38400","57600","115200","230400","460800","921600"};

    RingSet ring;
    Telemetry latest{};
    bool haveLatest = false;

    float sp_pos = 0.0f, sp_vel = 0.0f, sp_torque = 0.0f;

    std::deque<std::string> console;
    bool showAllLines = true;
    bool pauseScroll  = false;
    static constexpr size_t kConsoleCap = 2000;

    bool recording = false;
    std::ofstream recFile;
    char recPath[256] = "telem.csv";

    std::string statusMsg = "No Connection";
    ImVec4 statusColor = ImVec4(1.0f, 0.70f, 0.28f, 1.0f);

    char runDuration[16] = "10";
    char customCmd[128] = "";
    int fbkIdx = 0;   // MAG_ENCODER / HALL_SENSOR
    int modeIdx = 1;  // OPENLOOP_BEMF / CLOSED_FOC / CLOSED_SINUSOIDAL

    PIDRow pidPos    { "Position Loop",   "POS",    ImVec4(0.78f, 0.49f, 1.00f, 1.0f) };
    PIDRow pidVel    { "Velocity Loop",   "VEL",    ImVec4(0.00f, 0.83f, 0.67f, 1.0f) };
    PIDRow pidTorque { "Torque (Iq) Loop","TORQUE", ImVec4(1.00f, 0.70f, 0.28f, 1.0f) };
    PIDRow pidFlux   { "Flux (Id) Loop",  "FLUX",   ImVec4(0.31f, 0.62f, 1.00f, 1.0f) };

    void setStatus(const std::string& msg, const char* level) {
        statusMsg = msg;
        if (!std::strcmp(level, "ok"))   statusColor = ImVec4(0.0f, 0.83f, 0.67f, 1.0f);
        if (!std::strcmp(level, "warn")) statusColor = ImVec4(1.0f, 0.70f, 0.28f, 1.0f);
        if (!std::strcmp(level, "err"))  statusColor = ImVec4(1.0f, 0.30f, 0.43f, 1.0f);
    }

    void sendSerial(const std::string& cmd) {
        if (connected && worker.running()) {
            worker.writeLine(cmd);
            std::string trimmed = cmd;
            while (!trimmed.empty() && (trimmed.back() == '\n' || trimmed.back() == '\r'))
                trimmed.pop_back();
            setStatus("TX -> " + trimmed, "ok");
        } else {
            setStatus("No active connection", "warn");
        }
    }

    void refreshPorts() {
        ports = SerialPort::listPorts();
        std::sort(ports.begin(), ports.end(),
                  [](const PortInfo& a, const PortInfo& b) { return a.device < b.device; });
        portIdx = 0;
    }

    void connect() {
        if (ports.empty()) { setStatus("Select a valid port", "err"); return; }
        const std::string device = ports[portIdx].device;
        const int baud = std::atoi(kBauds[baudIdx]);
        worker.start(device, baud);
        connected = true;
        setStatus("Connected to " + device + " @ " + kBauds[baudIdx] + " baud", "ok");
    }

    void disconnect() {
        worker.stop();
        connected = false;
        setStatus("Disconnected", "warn");
        if (recording) toggleRecording();
    }

    void toggleRecording() {
        if (!recording) {
            std::time_t t = std::time(nullptr);
            std::tm tmv{};
#if defined(_WIN32)
            localtime_s(&tmv, &t);
#else
            localtime_r(&t, &tmv);
#endif
            std::snprintf(recPath, sizeof(recPath), "telem_%04d%02d%02d_%02d%02d%02d.csv",
                          tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                          tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
            recFile.open(recPath);
            if (!recFile.is_open()) { setStatus("Could not open CSV file for writing", "err"); return; }
            recFile << "timestamp,pvdd,iq,id,i3,a,b,c,nfault,noctw,ntc,"
                        "m_ang,red_ang,m_rpm,red_rpm,sp_pos,sp_vel,sp_torque\n";
            recording = true;
            setStatus(std::string("Recording -> ") + recPath, "ok");
        } else {
            recording = false;
            if (recFile.is_open()) recFile.close();
            setStatus("Recording stopped", "warn");
        }
    }

    void onFrame(const Telemetry& d) {
        ring.pvdd.push(d.pvdd);       ring.iq.push(d.iq);
        ring.id_.push(d.id_);         ring.i3.push(d.i3);
        ring.m_rpm.push(d.m_rpm);     ring.red_rpm.push(d.red_rpm);
        ring.ntc.push(d.ntc);         ring.m_ang.push(d.m_ang);
        ring.red_ang.push(d.red_ang); ring.a.push(d.a);
        ring.b.push(d.b);             ring.c.push(d.c);
        ring.sp_pos.push(sp_pos);     ring.sp_vel.push(sp_vel);
        ring.sp_torque.push(sp_torque);

        latest = d;
        haveLatest = true;

        if (recording && recFile.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()).count();
            recFile << ms << ',' << d.pvdd << ',' << d.iq << ',' << d.id_ << ',' << d.i3 << ','
                    << d.a << ',' << d.b << ',' << d.c << ',' << d.nfault << ',' << d.noctw << ','
                    << d.ntc << ',' << d.m_ang << ',' << d.red_ang << ',' << d.m_rpm << ','
                    << d.red_rpm << ',' << sp_pos << ',' << sp_vel << ',' << sp_torque << '\n';
        }
    }

    void onLine(const std::string& line) {
        if (!showAllLines) return;
        console.push_back(line);
        while (console.size() > kConsoleCap) console.pop_front();
    }
};

constexpr const char* AppState::kBauds[];

// =====================================================================================
//  THEME
// =====================================================================================

static void applyDarkTheme() {
    ImGuiStyle& st = ImGui::GetStyle();
    st.WindowRounding = 6.0f;
    st.FrameRounding  = 5.0f;
    st.GrabRounding   = 6.0f;
    st.TabRounding    = 5.0f;
    st.FramePadding   = ImVec2(8, 5);
    st.ItemSpacing    = ImVec2(8, 6);

    ImVec4* c = st.Colors;
    c[ImGuiCol_WindowBg]        = ImVec4(0.055f, 0.067f, 0.090f, 1.0f);
    c[ImGuiCol_ChildBg]         = ImVec4(0.086f, 0.106f, 0.153f, 1.0f);
    c[ImGuiCol_FrameBg]         = ImVec4(0.086f, 0.106f, 0.153f, 1.0f);
    c[ImGuiCol_FrameBgHovered]  = ImVec4(0.122f, 0.157f, 0.251f, 1.0f);
    c[ImGuiCol_FrameBgActive]   = ImVec4(0.122f, 0.157f, 0.251f, 1.0f);
    c[ImGuiCol_TitleBg]         = ImVec4(0.031f, 0.043f, 0.063f, 1.0f);
    c[ImGuiCol_TitleBgActive]   = ImVec4(0.031f, 0.043f, 0.063f, 1.0f);
    c[ImGuiCol_MenuBarBg]       = ImVec4(0.031f, 0.043f, 0.063f, 1.0f);
    c[ImGuiCol_Header]          = ImVec4(0.102f, 0.125f, 0.188f, 1.0f);
    c[ImGuiCol_HeaderHovered]   = ImVec4(0.122f, 0.157f, 0.251f, 1.0f);
    c[ImGuiCol_Button]          = ImVec4(0.102f, 0.125f, 0.188f, 1.0f);
    c[ImGuiCol_ButtonHovered]   = ImVec4(0.122f, 0.157f, 0.251f, 1.0f);
    c[ImGuiCol_ButtonActive]    = ImVec4(0.082f, 0.110f, 0.180f, 1.0f);
    c[ImGuiCol_Tab]             = ImVec4(0.086f, 0.106f, 0.153f, 1.0f);
    c[ImGuiCol_TabHovered]      = ImVec4(0.102f, 0.125f, 0.188f, 1.0f);
    c[ImGuiCol_TabActive]       = ImVec4(0.055f, 0.067f, 0.090f, 1.0f);
    c[ImGuiCol_Border]          = ImVec4(0.122f, 0.145f, 0.208f, 1.0f);
    c[ImGuiCol_Text]            = ImVec4(0.784f, 0.804f, 0.847f, 1.0f);
    c[ImGuiCol_SliderGrab]      = ImVec4(0.0f, 0.83f, 0.67f, 1.0f);
    c[ImGuiCol_SliderGrabActive}= ImVec4(0.0f, 0.83f, 0.67f, 1.0f);
    c[ImGuiCol_CheckMark]       = ImVec4(0.0f, 0.83f, 0.67f, 1.0f);
    c[ImGuiCol_PlotLines]       = ImVec4(0.0f, 0.83f, 0.67f, 1.0f);

    ImPlot::StyleColorsDark();
    ImPlotStyle& ps = ImPlot::GetStyle();
    ps.PlotPadding = ImVec2(8, 8);
    ps.LineWeight = 1.8f;
}

// =====================================================================================
//  PLOT HELPER  (mirrors Python's make_plot(): title + N series over the rolling window)
// =====================================================================================

struct SeriesSpec { const char* label; const RingSeries* data; ImVec4 color; bool dashed = false; };

static void drawTelemetryPlot(const char* id, const char* title, const char* ylabel,
                               const std::vector<SeriesSpec>& series, float height,
                               float thresholdY = NAN, const char* thresholdLabel = nullptr) {
    ImGui::TextColored(ImVec4(0.48f, 0.51f, 0.60f, 1.0f), "%s", title);
    if (ImPlot::BeginPlot(id, ImVec2(-1, height), ImPlotFlags_NoTitle)) {
        ImPlot::SetupAxes("sample", ylabel, ImPlotAxisFlags_NoTickLabels, 0);
        for (const auto& s : series) {
            ImPlot::PushStyleColor(ImPlotCol_Line, s.color);
            if (s.dashed) ImPlot::SetNextLineStyle(s.color, 1.5f);
            ImPlot::PlotLine(s.label, s.data->data(), s.data->size(), 1.0, 0.0,
                              0, s.data->offset());
            ImPlot::PopStyleColor();
        }
        if (!std::isnan(thresholdY)) {
            double xs[2] = {0.0, static_cast<double>(PLOT_LEN)};
            double ys[2] = {thresholdY, thresholdY};
            ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1.0f);
            ImPlot::PlotLine(thresholdLabel ? thresholdLabel : "limit", xs, ys, 2);
            ImPlot::PopStyleVar();
        }
        ImPlot::EndPlot();
    }
}

// =====================================================================================
//  MAIN
// =====================================================================================

int main() {
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit failed\n"); return 1; }

    const char* glslVersion = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1340, 880, "Motor Monitor  //  ESP32-S3 Controller (C++)",
                                           nullptr, nullptr);
    if (!window) { std::fprintf(stderr, "glfwCreateWindow failed\n"); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    applyDarkTheme();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    AppState app;
    app.refreshPorts();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ---- drain background serial worker ----
        if (app.connected) {
            std::vector<Telemetry> frames;
            std::vector<std::string> lines, errors;
            std::string fatal;
            app.worker.drain(frames, lines, errors, fatal);
            for (auto& f : frames) app.onFrame(f);
            for (auto& l : lines)  app.onLine(l);
            for (auto& e : errors) { app.setStatus(e, "warn"); app.console.push_back("[WARN] " + e); }
            if (!fatal.empty()) {
                app.setStatus(fatal, "err");
                app.console.push_back("[FATAL] " + fatal);
                app.disconnect();
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("MotorMonitorRoot", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoBringToFrontOnFocus);

        // ---------------- TOOLBAR ----------------
        ImGui::TextColored(ImVec4(0.0f, 0.83f, 0.67f, 1.0f), "ASTURION");
        ImGui::SameLine(); ImGui::TextDisabled("//");
        ImGui::SameLine(); ImGui::TextUnformatted("Port:");
        ImGui::SameLine(); ImGui::SetNextItemWidth(180);
        {
            std::string preview = app.ports.empty() ? "(No ports detected)" :
                                   (app.ports[app.portIdx].device + " - " + app.ports[app.portIdx].description);
            if (ImGui::BeginCombo("##port", preview.c_str())) {
                for (int i = 0; i < (int)app.ports.size(); i++) {
                    bool sel = (i == app.portIdx);
                    std::string label = app.ports[i].device + " - " + app.ports[i].description;
                    if (ImGui::Selectable(label.c_str(), sel)) app.portIdx = i;
                }
                ImGui::EndCombo();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) app.refreshPorts();

        ImGui::SameLine(); ImGui::TextUnformatted("Baud:");
        ImGui::SameLine(); ImGui::SetNextItemWidth(100);
        ImGui::Combo("##baud", &app.baudIdx, AppState::kBauds, 8);

        ImGui::SameLine();
        ImGui::BeginDisabled(app.connected);
        if (ImGui::Button("Connect")) app.connect();
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginDisabled(!app.connected);
        if (ImGui::Button("Disconnect")) app.disconnect();
        ImGui::EndDisabled();

        ImGui::SameLine(0, 30);
        ImGui::TextColored(app.connected ? ImVec4(0,0.83f,0.67f,1) : ImVec4(1,0.3f,0.43f,1),
                            app.connected ? "* ONLINE" : "* OFFLINE");

        ImGui::SameLine(ImGui::GetWindowWidth() - 180);
        ImGui::PushStyleColor(ImGuiCol_Button, app.recording ? ImVec4(0.24f,0.13f,0,1) : ImVec4(0.1f,0.12f,0.19f,1));
        if (ImGui::Button(app.recording ? "Stop Recording" : "Save CSV", ImVec2(160, 0)))
            app.toggleRecording();
        ImGui::PopStyleColor();

        ImGui::Separator();

        // ---------------- TABS ----------------
        if (ImGui::BeginTabBar("MainTabs")) {

            // ===== CONTROL =====
            if (ImGui::BeginTabItem("Control")) {
                ImGui::Columns(2, "control_cols", false);
                ImGui::SetColumnWidth(0, 440);

                ImGui::BeginChild("left_panel", ImVec2(0, 0), false);

                if (ImGui::CollapsingHeader("Motor Commands", ImGuiTreeNodeFlags_DefaultOpen)) {
                    if (ImGui::Button("ALIGN", ImVec2(90, 30))) app.sendSerial("CMD:ALIGN\n");
                    ImGui::SameLine();
                    if (ImGui::Button("RUN", ImVec2(90, 30))) {
                        std::string dur = app.runDuration;
                        app.sendSerial(dur.empty() ? "CMD:RUN\n" : ("CMD:RUN:" + dur + "\n"));
                    }
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(80);
                    ImGui::InputText("dur(s)", app.runDuration, sizeof(app.runDuration));

                    if (ImGui::Button("IDLE", ImVec2(90, 30))) app.sendSerial("CMD:IDLE\n");
                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.24f, 0.05f, 0.05f, 1));
                    if (ImGui::Button("BRAKE", ImVec2(190, 30))) app.sendSerial("CMD:BRAKE\n");
                    ImGui::PopStyleColor();

                    ImGui::Spacing();
                    ImGui::TextUnformatted("Feedback:");
                    ImGui::SameLine(120); ImGui::SetNextItemWidth(200);
                    static const char* fbkOpts[] = {"MAG_ENCODER", "HALL_SENSOR"};
                    if (ImGui::Combo("##fbk", &app.fbkIdx, fbkOpts, 2))
                        app.sendSerial(std::string("CMD:FBK:") + fbkOpts[app.fbkIdx] + "\n");

                    ImGui::TextUnformatted("Mode:");
                    ImGui::SameLine(120); ImGui::SetNextItemWidth(200);
                    static const char* modeOpts[] = {"OPENLOOP_BEMF", "CLOSED_FOC", "CLOSED_SINUSOIDAL"};
                    if (ImGui::Combo("##mode", &app.modeIdx, modeOpts, 3))
                        app.sendSerial(std::string("CMD:CTRL:") + modeOpts[app.modeIdx] + "\n");
                }

                if (ImGui::CollapsingHeader("Setpoints", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::SetNextItemWidth(150);
                    ImGui::InputFloat("Position (deg)", &app.sp_pos, 0, 0, "%.2f");
                    ImGui::SameLine();
                    if (ImGui::Button("Set##pos")) app.sendSerial(
                        "CMD:SP_POS:" + std::to_string(app.sp_pos) + "\n");

                    ImGui::SetNextItemWidth(150);
                    ImGui::InputFloat("Velocity (rpm)", &app.sp_vel, 0, 0, "%.2f");
                    ImGui::SameLine();
                    if (ImGui::Button("Set##vel")) app.sendSerial(
                        "CMD:SP_VEL:" + std::to_string(app.sp_vel) + "\n");

                    ImGui::SetNextItemWidth(150);
                    ImGui::InputFloat("Torque (A)", &app.sp_torque, 0, 0, "%.2f");
                    ImGui::SameLine();
                    if (ImGui::Button("Set##tor")) app.sendSerial(
                        "CMD:SP_TOR:" + std::to_string(app.sp_torque) + "\n");
                }

                if (ImGui::CollapsingHeader("Custom UART Command", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::SetNextItemWidth(280);
                    bool enter = ImGui::InputText("##customcmd", app.customCmd, sizeof(app.customCmd),
                                                   ImGuiInputTextFlags_EnterReturnsTrue);
                    ImGui::SameLine();
                    if (ImGui::Button("Send##custom") || enter) {
                        if (app.customCmd[0] != '\0') {
                            app.sendSerial(std::string(app.customCmd) + "\n");
                            app.customCmd[0] = '\0';
                        }
                    }
                }

                if (ImGui::CollapsingHeader("PID Tuning", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::TextDisabled("Sends CMD:PID_<LOOP>_K<X>:<val> on Send");
                    std::string cmd;
                    if (!(cmd = drawPIDRow(app.pidPos)).empty())    app.sendSerial(cmd);
                    ImGui::Separator();
                    if (!(cmd = drawPIDRow(app.pidVel)).empty())    app.sendSerial(cmd);
                    ImGui::Separator();
                    if (!(cmd = drawPIDRow(app.pidTorque)).empty()) app.sendSerial(cmd);
                    ImGui::Separator();
                    if (!(cmd = drawPIDRow(app.pidFlux)).empty())   app.sendSerial(cmd);
                }

                ImGui::EndChild();
                ImGui::NextColumn();

                ImGui::BeginChild("right_panel", ImVec2(0, 0), false);
                drawTelemetryPlot("##ctrl_pos", "Reducer Position", "degrees", {
                    {"Measured", &app.ring.red_ang, ImVec4(0.78f,0.49f,1.0f,1.0f)},
                    {"Setpoint", &app.ring.sp_pos,  ImVec4(0.78f,0.49f,1.0f,0.5f), true},
                }, 220);
                drawTelemetryPlot("##ctrl_vel", "Velocity", "RPM", {
                    {"Measured", &app.ring.m_rpm,  ImVec4(0.0f,0.83f,0.67f,1.0f)},
                    {"Setpoint", &app.ring.sp_vel, ImVec4(0.0f,0.83f,0.67f,0.5f), true},
                }, 220);
                drawTelemetryPlot("##ctrl_tor", "Torque (Phase Current)", "A", {
                    {"iq (measured)", &app.ring.iq,        ImVec4(1.0f,0.70f,0.28f,1.0f)},
                    {"Setpoint",      &app.ring.sp_torque, ImVec4(1.0f,0.70f,0.28f,0.5f), true},
                    {"id",            &app.ring.id_,       ImVec4(0.31f,0.62f,1.0f,1.0f)},
                }, 220);
                ImGui::EndChild();

                ImGui::Columns(1);
                ImGui::EndTabItem();
            }

            // ===== DASHBOARD =====
            if (ImGui::BeginTabItem("Dashboard")) {
                const Telemetry& d = app.latest;
                MetricCard cPvdd{"PVDD","V",d.pvdd,true,true,16,18};
                MetricCard cIq{"CURRENT Q","A",std::fabs(d.iq),true,true,20,35};
                MetricCard cId{"CURRENT D","A",std::fabs(d.id_),true,true,20,35};
                MetricCard cI3{"CURRENT 3","A",std::fabs(d.i3),true,true,20,35};
                MetricCard cRpm{"MOTOR RPM","rpm",d.m_rpm};
                MetricCard cRedRpm{"REDUCER RPM","rpm",d.red_rpm};
                MetricCard cNtc{"TEMPERATURE","C",d.ntc,true,true,60,80};

                drawMetricCard(cPvdd, "%.2f");    ImGui::SameLine();
                drawMetricCard(cIq, "%.2f");      ImGui::SameLine();
                drawMetricCard(cId, "%.2f");      ImGui::SameLine();
                drawMetricCard(cI3, "%.2f");
                drawMetricCard(cRpm, "%.1f");     ImGui::SameLine();
                drawMetricCard(cRedRpm, "%.2f");  ImGui::SameLine();
                drawMetricCard(cNtc, "%.1f");

                ImGui::Spacing();
                if (ImGui::CollapsingHeader("Digital Status", ImGuiTreeNodeFlags_DefaultOpen)) {
                    bool nfOk = d.nfault == 1, ocOk = d.noctw == 1;
                    ImGui::Text("ID: %d", d.id); ImGui::SameLine(150);
                    ImGui::TextColored(nfOk ? ImVec4(0,0.83f,0.67f,1) : ImVec4(1,0.3f,0.43f,1),
                                        "NFault: %s", nfOk ? "OK" : "FAULT");
                    ImGui::SameLine(300);
                    ImGui::TextColored(ocOk ? ImVec4(0,0.83f,0.67f,1) : ImVec4(1,0.7f,0.28f,1),
                                        "OC Warn: %s", ocOk ? "OK" : "WARN");
                    ImGui::Text("Motor angle: %.2f deg", d.m_ang);
                    ImGui::SameLine(300);
                    ImGui::Text("Reducer angle: %.2f deg", d.red_ang);
                }

                ImGui::Spacing();
                drawTelemetryPlot("##dash_rpm", "Motor RPM (quick view)", "rpm", {
                    {"Motor RPM",   &app.ring.m_rpm,   ImVec4(0.0f,0.83f,0.67f,1.0f)},
                    {"Reducer RPM", &app.ring.red_rpm, ImVec4(0.31f,0.62f,1.0f,1.0f)},
                }, 320);

                ImGui::EndTabItem();
            }

            // ===== CURRENTS =====
            if (ImGui::BeginTabItem("Currents")) {
                drawTelemetryPlot("##cur_phase", "Phase Currents", "Amps", {
                    {"iq", &app.ring.iq,  ImVec4(1.0f,0.30f,0.43f,1.0f)},
                    {"id", &app.ring.id_, ImVec4(1.0f,0.70f,0.28f,1.0f)},
                    {"I3", &app.ring.i3,  ImVec4(0.31f,0.62f,1.0f,1.0f)},
                }, 340);
                drawTelemetryPlot("##cur_adc", "ADC Phases (A, B, C)", "raw", {
                    {"A", &app.ring.a, ImVec4(1.0f,0.30f,0.43f,1.0f)},
                    {"B", &app.ring.b, ImVec4(0.31f,0.62f,1.0f,1.0f)},
                    {"C", &app.ring.c, ImVec4(0.0f,0.83f,0.67f,1.0f)},
                }, 340);
                ImGui::EndTabItem();
            }

            // ===== MECHANICAL =====
            if (ImGui::BeginTabItem("Mechanical")) {
                drawTelemetryPlot("##mech_rpm", "Angular Velocity", "RPM", {
                    {"Motor",   &app.ring.m_rpm,   ImVec4(0.0f,0.83f,0.67f,1.0f)},
                    {"Reducer", &app.ring.red_rpm, ImVec4(0.31f,0.62f,1.0f,1.0f)},
                }, 340);
                drawTelemetryPlot("##mech_ang", "Angle", "degrees", {
                    {"Motor angle",   &app.ring.m_ang,   ImVec4(0.78f,0.49f,1.0f,1.0f)},
                    {"Reducer angle", &app.ring.red_ang, ImVec4(1.0f,0.50f,0.31f,1.0f)},
                }, 340);
                ImGui::EndTabItem();
            }

            // ===== POWER · NTC =====
            if (ImGui::BeginTabItem("Power / NTC")) {
                drawTelemetryPlot("##pwr_pvdd", "Power Supply", "Volts", {
                    {"PVDD", &app.ring.pvdd, ImVec4(1.0f,0.70f,0.28f,1.0f)},
                }, 340, 16.0f, "overvoltage limit");
                drawTelemetryPlot("##pwr_ntc", "Temperature NTC", "C", {
                    {"NTC", &app.ring.ntc, ImVec4(1.0f,0.42f,0.42f,1.0f)},
                }, 340, 60.0f, "warn limit");
                ImGui::EndTabItem();
            }

            // ===== RAW CONSOLE =====
            if (ImGui::BeginTabItem("Raw Console")) {
                ImGui::Checkbox("Show all lines", &app.showAllLines);
                ImGui::SameLine();
                ImGui::Checkbox("Pause scroll", &app.pauseScroll);
                ImGui::SameLine();
                if (ImGui::Button("Clear")) app.console.clear();

                ImGui::BeginChild("console_scroll", ImVec2(0, 0), true,
                                   ImGuiWindowFlags_HorizontalScrollbar);
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.48f, 1.0f, 0.60f, 1.0f));
                for (auto& l : app.console) ImGui::TextUnformatted(l.c_str());
                ImGui::PopStyleColor();
                if (!app.pauseScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
                    ImGui::SetScrollHereY(1.0f);
                ImGui::EndChild();

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        // ---------------- STATUS BAR ----------------
        ImGui::Separator();
        ImGui::TextColored(app.statusColor, "  %s", app.statusMsg.c_str());

        ImGui::End();

        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.055f, 0.067f, 0.090f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    if (app.connected) app.disconnect();
    if (app.recFile.is_open()) app.recFile.close();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
