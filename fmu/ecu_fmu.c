#include <stddef.h>
#include <stdlib.h>

#if defined(_WIN32)
#define FMI_EXPORT __declspec(dllexport)
#else
#define FMI_EXPORT __attribute__((visibility("default")))
#endif

typedef double fmi2Real;
typedef int fmi2Integer;
typedef unsigned int fmi2ValueReference;
typedef int fmi2Status;
typedef void* fmi2Component;
typedef const char* fmi2String;
typedef int fmi2Boolean;
typedef void (*fmi2CallbackLogger)(void*, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
typedef void* (*fmi2CallbackAllocateMemory)(size_t, size_t);
typedef void (*fmi2CallbackFreeMemory)(void*);

typedef struct {
    fmi2CallbackLogger logger;
    fmi2CallbackAllocateMemory allocateMemory;
    fmi2CallbackFreeMemory freeMemory;
    void* stepFinished;
    void* componentEnvironment;
} fmi2CallbackFunctions;

typedef struct {
    fmi2Real throttle;
    fmi2Real brake;
    fmi2Real speed;
    fmi2Real rpm;
    fmi2Real coolant;
    fmi2Integer gear;
    fmi2Real soc;
    fmi2Real time;
} ecu_t;

FMI_EXPORT const char* fmi2GetTypesPlatform(void) { return "standard32"; }
FMI_EXPORT const char* fmi2GetVersion(void) { return "2.0"; }
FMI_EXPORT fmi2Component fmi2Instantiate(const char* instanceName, int fmuType, const char* guid,
    const char* resourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible,
    fmi2Boolean loggingOn) {
    (void)instanceName; (void)fmuType; (void)guid; (void)resourceLocation;
    (void)functions; (void)visible; (void)loggingOn;
    ecu_t* ecu = (ecu_t*)calloc(1, sizeof(ecu_t));
    if (ecu) { ecu->rpm = 800.0; ecu->coolant = 25.0; ecu->soc = 80.0; }
    return ecu;
}
FMI_EXPORT void fmi2FreeInstance(fmi2Component component) { free(component); }
FMI_EXPORT fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
    fmi2Real startTime, fmi2Boolean stopDefined, fmi2Real stopTime) {
    (void)toleranceDefined; (void)tolerance; (void)stopDefined; (void)stopTime;
    if (!c) return 3; ((ecu_t*)c)->time = startTime; return 0;
}
FMI_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component c) { return c ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component c) { return c ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2Terminate(fmi2Component c) { return c ? 0 : 3; }
FMI_EXPORT fmi2Status fmi2Reset(fmi2Component c) {
    if (!c) return 3; ecu_t* e = (ecu_t*)c; e->throttle = e->brake = e->speed = 0; e->rpm = 800; e->coolant = 25; e->gear = 0; e->soc = 80; e->time = 0; return 0;
}
FMI_EXPORT fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentTime, fmi2Real stepSize, fmi2Boolean noSetStatePrior) {
    (void)currentTime; (void)noSetStatePrior;
    if (!c || stepSize < 0) return 3;
    ecu_t* e = (ecu_t*)c;
    e->speed += (e->throttle * 0.06 - e->brake * 0.12 - e->speed * 0.01) * stepSize;
    if (e->speed < 0) e->speed = 0;
    if (e->speed > 120) e->speed = 120;
    e->rpm = 800 + e->speed * 42;
    e->coolant += (95 - e->coolant) * 0.01 * stepSize;
    e->soc -= 0.001 * stepSize;
    e->gear = e->speed > 60 ? 5 : e->speed > 40 ? 4 : e->speed > 25 ? 3 : e->speed > 10 ? 2 : e->speed > 0 ? 1 : 0;
    e->time += stepSize;
    return 0;
}
FMI_EXPORT fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference* vr, size_t n, const fmi2Real* v) {
    if (!c) return 3; ecu_t* e = (ecu_t*)c; for (size_t i=0;i<n;i++) { if(vr[i]==1)e->throttle=v[i]; else if(vr[i]==2)e->brake=v[i]; } return 0;
}
FMI_EXPORT fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference* vr, size_t n, fmi2Real* v) {
    if (!c) return 3; ecu_t* e = (ecu_t*)c; for (size_t i=0;i<n;i++) { if(vr[i]==1)v[i]=e->throttle; else if(vr[i]==2)v[i]=e->brake; else if(vr[i]==3)v[i]=e->speed; else if(vr[i]==4)v[i]=e->rpm; else if(vr[i]==5)v[i]=e->coolant; else if(vr[i]==7)v[i]=e->soc; else v[i]=0; } return 0;
}
FMI_EXPORT fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference* vr, size_t n, const fmi2Integer* v) { (void)c;(void)vr;(void)n;(void)v; return 0; }
FMI_EXPORT fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference* vr, size_t n, fmi2Integer* v) { if(!c)return 3; ecu_t*e=(ecu_t*)c; for(size_t i=0;i<n;i++)v[i]=vr[i]==6?e->gear:0; return 0; }
