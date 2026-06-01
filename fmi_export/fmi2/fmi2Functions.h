// FMI 2.0 — function declarations (Co-Simulation subset).
#ifndef fmi2Functions_h
#define fmi2Functions_h

#include "fmi2TypesPlatform.h"
#include <stddef.h>

#ifdef _WIN32
  #define FMI2_EXPORT __declspec(dllexport)
#else
  #define FMI2_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define fmi2Version "2.0"

// ---- Inquire / common ----
FMI2_EXPORT const char*  fmi2GetVersion(void);
FMI2_EXPORT const char*  fmi2GetTypesPlatform(void);

FMI2_EXPORT fmi2Status   fmi2SetDebugLogging(fmi2Component, fmi2Boolean,
                                              size_t, const fmi2String[]);

// ---- Creation / destruction ----
FMI2_EXPORT fmi2Component fmi2Instantiate(
    fmi2String  instanceName,
    fmi2Type    fmuType,
    fmi2String  fmuGUID,
    fmi2String  fmuResourceLocation,
    const fmi2CallbackFunctions* functions,
    fmi2Boolean visible,
    fmi2Boolean loggingOn);

FMI2_EXPORT void          fmi2FreeInstance(fmi2Component);

// ---- Setup / state ----
FMI2_EXPORT fmi2Status    fmi2SetupExperiment(fmi2Component, fmi2Boolean,
                                                fmi2Real, fmi2Real, fmi2Boolean,
                                                fmi2Real);
FMI2_EXPORT fmi2Status    fmi2EnterInitializationMode(fmi2Component);
FMI2_EXPORT fmi2Status    fmi2ExitInitializationMode(fmi2Component);
FMI2_EXPORT fmi2Status    fmi2Reset(fmi2Component);
FMI2_EXPORT fmi2Status    fmi2Terminate(fmi2Component);

// ---- I/O ----
FMI2_EXPORT fmi2Status    fmi2GetReal   (fmi2Component, const fmi2ValueReference[], size_t, fmi2Real[]);
FMI2_EXPORT fmi2Status    fmi2GetInteger(fmi2Component, const fmi2ValueReference[], size_t, fmi2Integer[]);
FMI2_EXPORT fmi2Status    fmi2GetBoolean(fmi2Component, const fmi2ValueReference[], size_t, fmi2Boolean[]);
FMI2_EXPORT fmi2Status    fmi2GetString (fmi2Component, const fmi2ValueReference[], size_t, fmi2String[]);
FMI2_EXPORT fmi2Status    fmi2SetReal   (fmi2Component, const fmi2ValueReference[], size_t, const fmi2Real[]);
FMI2_EXPORT fmi2Status    fmi2SetInteger(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[]);
FMI2_EXPORT fmi2Status    fmi2SetBoolean(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Boolean[]);
FMI2_EXPORT fmi2Status    fmi2SetString (fmi2Component, const fmi2ValueReference[], size_t, const fmi2String[]);

// ---- Co-simulation only ----
FMI2_EXPORT fmi2Status    fmi2SetRealInputDerivatives(fmi2Component,
    const fmi2ValueReference[], size_t, const fmi2Integer[], const fmi2Real[]);
FMI2_EXPORT fmi2Status    fmi2GetRealOutputDerivatives(fmi2Component,
    const fmi2ValueReference[], size_t, const fmi2Integer[], fmi2Real[]);
FMI2_EXPORT fmi2Status    fmi2DoStep(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);
FMI2_EXPORT fmi2Status    fmi2CancelStep(fmi2Component);
FMI2_EXPORT fmi2Status    fmi2GetStatus(fmi2Component, fmi2StatusKind, fmi2Status*);
FMI2_EXPORT fmi2Status    fmi2GetRealStatus(fmi2Component, fmi2StatusKind, fmi2Real*);
FMI2_EXPORT fmi2Status    fmi2GetIntegerStatus(fmi2Component, fmi2StatusKind, fmi2Integer*);
FMI2_EXPORT fmi2Status    fmi2GetBooleanStatus(fmi2Component, fmi2StatusKind, fmi2Boolean*);
FMI2_EXPORT fmi2Status    fmi2GetStringStatus(fmi2Component, fmi2StatusKind, fmi2String*);

#ifdef __cplusplus
}
#endif
#endif
