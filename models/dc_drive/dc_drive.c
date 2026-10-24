/* dc_drive: the reference FMU of Lagrange's FMU host (docs/design/formats.md section 6).
 *
 * A brushed DC motor with its drive, FMI 3.0 co-simulation: a PI current loop tuned by
 * pole-zero cancellation turns the torque command into a voltage, limited by the bus (an
 * H bridge) and the current limit; the armature current is advanced exactly for the voltage held
 * over the step. The same algorithm as Lagrange's built-in brushed model (Models.cpp), so a
 * test can check the FMU path against it. One step per fmi3DoStep.
 *
 * SPDX-License-Identifier: Apache-2.0 */
#include "fmi3Functions.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define INSTANTIATION_TOKEN "{6d1f3c38-6a3e-4c2a-9a55-1d4b8b2f6c01}"

/* Value references, as in modelDescription.xml. */
enum {
    VR_R = 1,
    VR_L,
    VR_KT,
    VR_IMAX,
    VR_ALPHA,
    VR_TREF,
    VR_WC, /* parameters */
    VR_OMEGA = 10,
    VR_TAU_CMD,
    VR_V_BUS,
    VR_T_WINDING, /* inputs */
    VR_TAU = 20,
    VR_I_BUS,
    VR_P_LOSS,
    VR_I_A, /* outputs */
    VR_COUNT = 24
};

typedef struct {
    double v[VR_COUNT]; /* every variable by value reference */
    double current;     /* A, armature */
    double integral;    /* V, the PI integrator */
} State;

typedef struct {
    State s;
    fmi3InstanceEnvironment environment;
    fmi3LogMessageCallback log;
} Drive;

static void set_defaults(State* s) {
    memset(s, 0, sizeof(*s));
    s->v[VR_R] = 0.5;
    s->v[VR_L] = 0.0005;
    s->v[VR_KT] = 0.05;
    s->v[VR_IMAX] = 10.0;
    s->v[VR_ALPHA] = 0.00393;
    s->v[VR_TREF] = 298.15;
    s->v[VR_WC] = 6283.0;
    s->v[VR_T_WINDING] = 298.15;
}

const char* fmi3GetVersion(void) {
    return fmi3Version;
}

fmi3Status fmi3SetDebugLogging(fmi3Instance instance, fmi3Boolean loggingOn, size_t nCategories,
                               const fmi3String categories[]) {
    (void)instance;
    (void)loggingOn;
    (void)nCategories;
    (void)categories;
    return fmi3OK;
}

fmi3Instance fmi3InstantiateModelExchange(fmi3String instanceName, fmi3String instantiationToken,
                                          fmi3String resourcePath, fmi3Boolean visible,
                                          fmi3Boolean loggingOn,
                                          fmi3InstanceEnvironment instanceEnvironment,
                                          fmi3LogMessageCallback logMessage) {
    (void)instanceName;
    (void)instantiationToken;
    (void)resourcePath;
    (void)visible;
    (void)loggingOn;
    (void)instanceEnvironment;
    (void)logMessage;
    return NULL; /* co-simulation only */
}

fmi3Instance fmi3InstantiateCoSimulation(
    fmi3String instanceName, fmi3String instantiationToken, fmi3String resourcePath,
    fmi3Boolean visible, fmi3Boolean loggingOn, fmi3Boolean eventModeUsed,
    fmi3Boolean earlyReturnAllowed, const fmi3ValueReference requiredIntermediateVariables[],
    size_t nRequiredIntermediateVariables, fmi3InstanceEnvironment instanceEnvironment,
    fmi3LogMessageCallback logMessage, fmi3IntermediateUpdateCallback intermediateUpdate) {
    (void)instanceName;
    (void)resourcePath;
    (void)visible;
    (void)loggingOn;
    (void)earlyReturnAllowed;
    (void)requiredIntermediateVariables;
    (void)nRequiredIntermediateVariables;
    (void)intermediateUpdate;
    if (instantiationToken == NULL || strcmp(instantiationToken, INSTANTIATION_TOKEN) != 0 ||
        eventModeUsed) {
        return NULL;
    }
    Drive* d = calloc(1, sizeof(Drive));
    if (d == NULL) {
        return NULL;
    }
    set_defaults(&d->s);
    d->environment = instanceEnvironment;
    d->log = logMessage;
    return d;
}

fmi3Instance fmi3InstantiateScheduledExecution(
    fmi3String instanceName, fmi3String instantiationToken, fmi3String resourcePath,
    fmi3Boolean visible, fmi3Boolean loggingOn, fmi3InstanceEnvironment instanceEnvironment,
    fmi3LogMessageCallback logMessage, fmi3ClockUpdateCallback clockUpdate,
    fmi3LockPreemptionCallback lockPreemption, fmi3UnlockPreemptionCallback unlockPreemption) {
    (void)instanceName;
    (void)instantiationToken;
    (void)resourcePath;
    (void)visible;
    (void)loggingOn;
    (void)instanceEnvironment;
    (void)logMessage;
    (void)clockUpdate;
    (void)lockPreemption;
    (void)unlockPreemption;
    return NULL;
}

void fmi3FreeInstance(fmi3Instance instance) {
    free(instance);
}

fmi3Status fmi3EnterInitializationMode(fmi3Instance instance, fmi3Boolean toleranceDefined,
                                       fmi3Float64 tolerance, fmi3Float64 startTime,
                                       fmi3Boolean stopTimeDefined, fmi3Float64 stopTime) {
    (void)instance;
    (void)toleranceDefined;
    (void)tolerance;
    (void)startTime;
    (void)stopTimeDefined;
    (void)stopTime;
    return fmi3OK;
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) {
    (void)instance;
    return fmi3OK;
}

fmi3Status fmi3EnterStepMode(fmi3Instance instance) {
    (void)instance;
    return fmi3OK;
}

fmi3Status fmi3Terminate(fmi3Instance instance) {
    (void)instance;
    return fmi3OK;
}

fmi3Status fmi3Reset(fmi3Instance instance) {
    set_defaults(&((Drive*)instance)->s);
    return fmi3OK;
}

fmi3Status fmi3GetFloat64(fmi3Instance instance, const fmi3ValueReference valueReferences[],
                          size_t nValueReferences, fmi3Float64 values[], size_t nValues) {
    const Drive* d = instance;
    if (nValues < nValueReferences) {
        return fmi3Error;
    }
    for (size_t i = 0; i < nValueReferences; ++i) {
        if (valueReferences[i] >= VR_COUNT) {
            return fmi3Error;
        }
        values[i] = d->s.v[valueReferences[i]];
    }
    return fmi3OK;
}

fmi3Status fmi3SetFloat64(fmi3Instance instance, const fmi3ValueReference valueReferences[],
                          size_t nValueReferences, const fmi3Float64 values[], size_t nValues) {
    Drive* d = instance;
    if (nValues < nValueReferences) {
        return fmi3Error;
    }
    for (size_t i = 0; i < nValueReferences; ++i) {
        if (valueReferences[i] >= VR_COUNT || valueReferences[i] >= VR_TAU) {
            return fmi3Error; /* outputs cannot be set */
        }
        d->s.v[valueReferences[i]] = values[i];
    }
    return fmi3OK;
}

/* One armature step: L di/dt = u - R i with u held, solved exactly. */
static double advance(double current, double u, double r, double l, double h) {
    if (l <= 0.0) {
        return r > 0.0 ? u / r : current;
    }
    if (r <= 0.0) {
        return current + u * h / l;
    }
    const double decay = exp(-r * h / l);
    return decay * current + (1.0 - decay) * u / r;
}

fmi3Status fmi3DoStep(fmi3Instance instance, fmi3Float64 currentCommunicationPoint,
                      fmi3Float64 communicationStepSize,
                      fmi3Boolean noSetFMUStatePriorToCurrentPoint,
                      fmi3Boolean* eventHandlingNeeded, fmi3Boolean* terminateSimulation,
                      fmi3Boolean* earlyReturn, fmi3Float64* lastSuccessfulTime) {
    (void)noSetFMUStatePriorToCurrentPoint;
    State* s = &((Drive*)instance)->s;
    const double h = communicationStepSize;
    double* v = s->v;
    *eventHandlingNeeded = fmi3False;
    *terminateSimulation = fmi3False;
    *earlyReturn = fmi3False;
    *lastSuccessfulTime = currentCommunicationPoint + h;
    const double bus = v[VR_V_BUS];
    if (bus <= 0.0) {
        s->current = s->integral = 0.0;
        v[VR_TAU] = v[VR_I_BUS] = v[VR_P_LOSS] = v[VR_I_A] = 0.0;
        return fmi3OK;
    }
    const double r = v[VR_R] * (1.0 + v[VR_ALPHA] * (v[VR_T_WINDING] - v[VR_TREF]));
    double ref = v[VR_TAU_CMD] / v[VR_KT];
    if (ref > v[VR_IMAX]) ref = v[VR_IMAX];
    if (ref < -v[VR_IMAX]) ref = -v[VR_IMAX];
    /* Bandwidth capped at 1/h: a faster discrete loop diverges. */
    const double wc = v[VR_WC] < 1.0 / h ? v[VR_WC] : 1.0 / h;
    const double error = ref - s->current;
    const double emf = v[VR_KT] * v[VR_OMEGA];
    double volts = v[VR_L] * wc * error + s->integral + emf;
    if (fabs(volts) > bus) {
        volts = volts > 0.0 ? bus : -bus; /* saturated: hold the integrator */
    } else {
        s->integral += v[VR_R] * wc * error * h;
    }
    const double start = s->current;
    s->current = advance(s->current, volts - emf, r, v[VR_L], h);
    const double mean = 0.5 * (start + s->current);
    v[VR_I_A] = s->current;
    v[VR_TAU] = v[VR_KT] * s->current;
    v[VR_I_BUS] = volts * mean / bus;
    v[VR_P_LOSS] = r * mean * mean;
    return fmi3OK;
}

/* FMU state: a copy of the instance's state, serialized as its bytes. */
fmi3Status fmi3GetFMUState(fmi3Instance instance, fmi3FMUState* FMUState) {
    State* copy = *FMUState != NULL ? *FMUState : malloc(sizeof(State));
    if (copy == NULL) {
        return fmi3Error;
    }
    *copy = ((Drive*)instance)->s;
    *FMUState = copy;
    return fmi3OK;
}

fmi3Status fmi3SetFMUState(fmi3Instance instance, fmi3FMUState FMUState) {
    ((Drive*)instance)->s = *(const State*)FMUState;
    return fmi3OK;
}

fmi3Status fmi3FreeFMUState(fmi3Instance instance, fmi3FMUState* FMUState) {
    (void)instance;
    free(*FMUState);
    *FMUState = NULL;
    return fmi3OK;
}

fmi3Status fmi3SerializedFMUStateSize(fmi3Instance instance, fmi3FMUState FMUState, size_t* size) {
    (void)instance;
    (void)FMUState;
    *size = sizeof(State);
    return fmi3OK;
}

fmi3Status fmi3SerializeFMUState(fmi3Instance instance, fmi3FMUState FMUState,
                                 fmi3Byte serializedState[], size_t size) {
    (void)instance;
    if (size < sizeof(State)) {
        return fmi3Error;
    }
    memcpy(serializedState, FMUState, sizeof(State));
    return fmi3OK;
}

fmi3Status fmi3DeserializeFMUState(fmi3Instance instance, const fmi3Byte serializedState[],
                                   size_t size, fmi3FMUState* FMUState) {
    (void)instance;
    if (size < sizeof(State)) {
        return fmi3Error;
    }
    State* copy = malloc(sizeof(State));
    if (copy == NULL) {
        return fmi3Error;
    }
    memcpy(copy, serializedState, sizeof(State));
    *FMUState = copy;
    return fmi3OK;
}

#include "stubs.inc"
