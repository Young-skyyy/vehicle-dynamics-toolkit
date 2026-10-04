// FMI 2.0 Co-Simulation adapter of the SAME native plant used by ROS2/replay.
#include "dynamics_core.hpp"
#include <cstring>
#include <new>

#ifdef _WIN32
#define FMI_EXPORT extern "C" __declspec(dllexport)
#else
#define FMI_EXPORT extern "C" __attribute__((visibility("default")))
#endif
using fmi2Component = void*;
using fmi2Real = double;
using fmi2Integer = int;
using fmi2Boolean = int;
using fmi2ValueReference = unsigned int;
using fmi2String = const char*;
using fmi2Status = int;
struct fmi2CallbackFunctions {
    void (*logger)(void*, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
    void* (*allocateMemory)(size_t, size_t);
    void (*freeMemory)(void*);
    void (*stepFinished)(void*, fmi2Status);
    void* componentEnvironment;
};
namespace {
constexpr const char* GUID = "{vehicle-dynamics-toolkit-vehicle-plant-v2}";
enum class Phase { instantiated, initializing, running, terminated };
struct Plant {
    vehicle_dynamics::Model model{};
    double throttle = 0, brake = 0, steer = 0, initial = 0, start = 0, stop = 0;
    bool stop_defined = false;
    Phase phase = Phase::instantiated;
    void (*free_memory)(void*) = nullptr;
};
bool real_ref(unsigned int ref) { return ref >= 1 && ref <= 19 && ref != 6; }
double real_value(const Plant& p, unsigned int ref) {
    const auto& s = p.model.state();
    switch (ref) {
    case 1: return p.throttle; case 2: return p.brake;
    case 3: return s.vx * 3.6; case 4: return s.engine_rpm;
    case 5: return 25; case 7: return 80; case 8: return p.steer;
    case 9: return s.vx; case 10: return s.vy; case 11: return s.ax;
    case 12: return s.ay; case 13: return s.yaw_rate; case 14: return s.heading;
    case 15: return s.position_x; case 16: return s.position_y;
    case 17: return s.t; case 18: return s.engine_torque; case 19: return p.initial;
    default: return 0;
    }
}
}
FMI_EXPORT const char* fmi2GetTypesPlatform() { return "default"; }
FMI_EXPORT const char* fmi2GetVersion() { return "2.0"; }
FMI_EXPORT fmi2Component fmi2Instantiate(fmi2String name, int type, fmi2String guid,
    fmi2String, const fmi2CallbackFunctions* callbacks, fmi2Boolean, fmi2Boolean) {
    if (!name || !*name || type != 1 || !guid || std::strcmp(guid, GUID)) return nullptr;
    if (callbacks && (!callbacks->allocateMemory || !callbacks->freeMemory)) return nullptr;
    void* memory = nullptr;
    try {
        if (callbacks) {
            memory = callbacks->allocateMemory(1, sizeof(Plant));
            if (!memory) return nullptr;
            auto* p = new(memory) Plant;
            p->free_memory = callbacks->freeMemory;
            return p;
        }
        return new Plant;
    } catch (...) {
        if (memory) callbacks->freeMemory(memory);
        return nullptr;
    }
}
FMI_EXPORT void fmi2FreeInstance(fmi2Component c) {
    auto* p = static_cast<Plant*>(c);
    if (!p) return;
    if (p->free_memory) { auto release = p->free_memory; p->~Plant(); release(p); }
    else delete p;
}
FMI_EXPORT fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean tolerance_defined,
    double tolerance, double start, fmi2Boolean stop_defined, double stop) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase != Phase::instantiated || !std::isfinite(start) || start < 0 ||
        (tolerance_defined && (!std::isfinite(tolerance) || tolerance <= 0)) ||
        (stop_defined && (!std::isfinite(stop) || stop < start))) return 3;
    try {
        p->model = vehicle_dynamics::Model({}, p->initial, start);
        p->start = start; p->stop = stop; p->stop_defined = stop_defined != 0;
        return 0;
    } catch (...) { return 3; }
}
FMI_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase != Phase::instantiated) return 3;
    p->phase = Phase::initializing; return 0;
}
FMI_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase != Phase::initializing) return 3;
    p->phase = Phase::running; return 0;
}
FMI_EXPORT fmi2Status fmi2Terminate(fmi2Component c) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase != Phase::running) return 3;
    p->phase = Phase::terminated; return 0;
}
FMI_EXPORT fmi2Status fmi2Reset(fmi2Component c) {
    auto* p = static_cast<Plant*>(c);
    if (!p) return 3;
    try { auto release = p->free_memory; *p = Plant{}; p->free_memory = release; return 0; }
    catch (...) { return 3; }
}
FMI_EXPORT fmi2Status fmi2DoStep(fmi2Component c, double current, double dt, fmi2Boolean) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase != Phase::running || !std::isfinite(current) || !std::isfinite(dt) ||
        dt <= 0 || dt > .1 || std::abs(current - p->model.state().t) > 1e-9 ||
        (p->stop_defined && current + dt > p->stop + 1e-9)) return 3;
    try { p->model.step(dt, p->throttle, p->brake, p->steer); return 0; }
    catch (...) { return 3; }
}
FMI_EXPORT fmi2Status fmi2SetReal(fmi2Component c, const unsigned int* refs, size_t n, const double* values) {
    auto* p = static_cast<Plant*>(c);
    if (!p || p->phase == Phase::terminated || (n && (!refs || !values))) return 3;
    double throttle = p->throttle, brake = p->brake, steer = p->steer, initial = p->initial;
    bool set_initial = false;
    // Validate whole batch before committing; an invalid trailing ref cannot partially set input.
    for (size_t i = 0; i < n; ++i) {
        double value = values[i];
        if (!std::isfinite(value)) return 3;
        switch (refs[i]) {
        case 1: if (value < 0 || value > 1) return 3; throttle = value; break;
        case 2: if (value < 0 || value > 1) return 3; brake = value; break;
        case 8: if (std::abs(value) > .7) return 3; steer = value; break;
        case 19:
            if (value < 0 || p->phase == Phase::running) return 3;
            initial = value; set_initial = true; break;
        default: return 3;
        }
    }
    try {
        if (set_initial) p->model = vehicle_dynamics::Model({}, initial, p->start);
        p->throttle = throttle; p->brake = brake; p->steer = steer; p->initial = initial;
        return 0;
    } catch (...) { return 3; }
}
FMI_EXPORT fmi2Status fmi2GetReal(fmi2Component c, const unsigned int* refs, size_t n, double* values) {
    auto* p = static_cast<Plant*>(c);
    if (!p || (n && (!refs || !values))) return 3;
    for (size_t i = 0; i < n; ++i) if (!real_ref(refs[i])) return 3;
    for (size_t i = 0; i < n; ++i) values[i] = real_value(*p, refs[i]);
    return 0;
}
FMI_EXPORT fmi2Status fmi2GetInteger(fmi2Component c, const unsigned int* refs, size_t n, int* values) {
    auto* p = static_cast<Plant*>(c);
    if (!p || (n && (!refs || !values))) return 3;
    for (size_t i = 0; i < n; ++i) if (refs[i] != 6) return 3;
    for (size_t i = 0; i < n; ++i) values[i] = p->model.state().gear;
    return 0;
}
// No writable integer/boolean/string variables or interpolation/asynchronous stepping.
FMI_EXPORT fmi2Status fmi2SetInteger(fmi2Component c, const unsigned int*, size_t n, const int*) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2SetBoolean(fmi2Component c, const unsigned int*, size_t n, const int*) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2GetBoolean(fmi2Component c, const unsigned int*, size_t n, int*) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2SetString(fmi2Component c, const unsigned int*, size_t n, const char* const*) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2GetString(fmi2Component c, const unsigned int*, size_t n, const char**) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2SetDebugLogging(fmi2Component c, int, size_t n, const char* const*) { return c && !n ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2SetRealInputDerivatives(fmi2Component, const unsigned int*, size_t, const int*, const double*) { return 3; }
FMI_EXPORT fmi2Status fmi2GetRealOutputDerivatives(fmi2Component, const unsigned int*, size_t, const int*, double*) { return 3; }
FMI_EXPORT fmi2Status fmi2CancelStep(fmi2Component) { return 3; }
FMI_EXPORT fmi2Status fmi2GetStatus(fmi2Component, int, int*) { return 3; }
FMI_EXPORT fmi2Status fmi2GetRealStatus(fmi2Component c, int kind, double* value) {
    auto* p = static_cast<Plant*>(c);
    if (!p || !value || kind != 2) return 3;
    *value = p->model.state().t; return 0;
}
FMI_EXPORT fmi2Status fmi2GetIntegerStatus(fmi2Component, int, int*) { return 3; }
FMI_EXPORT fmi2Status fmi2GetBooleanStatus(fmi2Component c, int kind, int* value) {
    auto* p = static_cast<Plant*>(c);
    if (!p || !value || kind != 3) return 3;
    *value = p->phase == Phase::terminated; return 0;
}
FMI_EXPORT fmi2Status fmi2GetStringStatus(fmi2Component, int, const char**) { return 3; }
