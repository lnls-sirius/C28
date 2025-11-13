/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file scope.h
 * @brief Scope module
 *
 * This module implements functions for Scope functionality, which serves as a
 * configurable buffer for signal acquisition.
 *
 * @author gabriel.brunheira
 * @date 01/04/2020
 *
 */

#ifndef SCOPE_H_
#define SCOPE_H_

#include <stdint.h>
#include "common/structs.h"
#include "common/timeslicer.h"

#define NUM_MAX_SCOPES      4

#define RUN_SCOPE(scp)  RUN_TIMESLICER(scp.timeslicer)  \
                            scp.p_run_scope(&scp);      \
                        END_TIMESLICER(scp.timeslicer)

typedef volatile struct scope_t scope_t;
struct scope_t
{

    buf_t           buffer;
    timeslicer_t    timeslicer;
    float           duration;
    float           *p_source;
    void            (*p_run_scope)(scope_t *p_scp);
};

inline void run_scope(scope_t *p_scp)
{
    /*********************************************/
    RUN_TIMESLICER(p_scp->timeslicer)
    /*********************************************/
        p_scp->p_run_scope(p_scp);
    /*********************************************/
    END_TIMESLICER(p_scp->timeslicer)
    /*********************************************/
}

extern void init_scope(scope_t *p_scp, float freq_base, float freq_sampling,
                       float *p_buf_start, uint16_t size, float *p_source,
                       void *p_run_scope);
extern void cfg_source_scope(scope_t *p_scp,float *p_source);
extern void cfg_freq_scope(scope_t *p_scp, float freq_sampling);
extern void cfg_duration_scope(scope_t *p_scp, float duration);
extern void enable_scope(scope_t *p_scp);
extern void disable_scope(scope_t *p_scp);
extern void reset_scope(scope_t *p_scp);
extern void run_scope_shared_ram(scope_t *p_scp);

#endif
