/* Temporary solution to make it easier to put the interrupt handling "side-by-side"
 * This file is included from e1000e_core.c, ~line 2204
 * Some form of refactoring is needed but not clear what the best solution is yet.
 */


/* TBD: For the IGB interrupt registers below we assume MSI-X mode
 * eg. we assume GPIE.Multiple_MSIX = 1
 * for now and ignore the alternative interpretation for legacy mode!
 */

/* igb EICR (Extended Interrupt Cause, spec p.352, 502)
 * W1C (set to clear) semantics
 */
static void
igb_set_eicr(E1000ECore *core, int index, uint32_t val)
{
    core->mac[EICR] &= ~val;
}

/* igb EICS (Extended Interrupt Cause Set, spec p.352, 502),
 * Set corresponding bits in EICR (Extended Interrupt Cause Register)
 */
static void
igb_set_eics(E1000ECore *core, int index, uint32_t val)
{
    uint32_t interrupting;
    core->mac[EICR] |= (val & E1000_EICR_MASK);

    interrupting = core->mac[EICR] & core->mac[EIMS];

    /* TBD:
     * - Loop through bits to trigger interrupts
     * - Modify according to EAIM
     */
    (void)interrupting;
}

/* igb EIMS (Extended Interrupt Mask Set, spec p.352, 503)
 * Enables interrupts by writing 1's.
 */
static void
igb_set_eims(E1000ECore *core, int index, uint32_t val)
{
    uint32_t eims = core->mac[EIMS];
    uint32_t eims_new = eims | (val & E1000_EICR_MASK);
    uint32_t triggered = eims_new & ~eims & core->mac[EICR];

    if (triggered) {
        core->mac[EIMS] |=  eims_new;
        /* TBD: Loop through set bits and generate interrupt.. */
    }
}

/* igb EIMC (Extended Interrupt Mask Clear, spec p.352, 504)
 * Clear corresponding bit in EIMS
 */
static void
igb_set_eimc(E1000ECore *core, int index, uint32_t val)
{
    core->mac[EIMS] &= ~(val & E1000_EICR_MASK);
}

/* igb EIAM (Extended Interrupt Auto Mask Enable, spec p.352, 504) */
static void
igb_set_eiam(E1000ECore *core, int index, uint32_t val)
{
    core->mac[EIAM] |= (val & E1000_EICR_MASK);
}

/* igb EIAC (Extended Interrupt Auto Clear, spec p.504) */
static void
igb_set_eiac(E1000ECore *core, int index, uint32_t val)
{
    core->mac[EIAC] |= (val & E1000_EICR_MASK);
}

#if 0
/* Interrupt handling for the igb
 * - regs differ substantially from the e1000e.
 * There's fixed positions in the IVAR regs for the queues, as for
 * e1000e, but they are calculated differently, according to
 * table 7-43, cause allocation,
 * and IVAR[0:7] definition - sec.8.8.13, p.513:
 */

#define IGB_IVAR_ENTRY_MASK   0xff
#define IGB_IVAR_ENTRY_VALID
#define IGB_IVAR_ENTRY_VEC_MASK (0x1f)
#define IGB_IVAR_ENTRY_VEC(x) ((x) & IGB_IVAR_ENTRY_VEC_MASK)


/* TBD: Needs the inverse functionality here vector -> entry! */

inline uint32_t igb_get_ivar_entry(E1000ECore *core, bool tx, uint16_t index)
{
    uint16_t regno = index < 8 ? index : index - 2;
    uint16_t i_tx = tx ? 1 : 0;
    uint16_t i_off = index < 8 ? i_tx : i_tx + 2;

    return (core->mac[I_IVAR + regno] << (i_off * 8)) & IGB_IVAR_ENTRY_MASK;
}

inline void igb_set_ivar_entry(E1000ECore *core, bool tx, uint16_t index, uint8_t value)
{
    uint16_t regno = index < 8 ? index : index - 2;
    uint16_t i_tx = tx ? 1 : 0;
    uint16_t i_off = index < 8 ? i_tx : i_tx + 2;
    uint16_t shift = i_off * 8;

    /* Mask off old value, then set new */
    uint32_t oldval = core->mac[I_IVAR + regno] & ~(IGB_IVAR_ENTRY_MASK << shift);
    core->mac[I_IVAR + regno] = oldval | ((value & IGB_IVAR_ENTRY_MASK) << shift);
}


static void
igb_msix_notify_one(E1000ECore *core, uint32_t cause, uint32_t int_cfg)
{
    uint32_t effective_eiac;

    if (IGB_IVAR_ENTRY_VALID(int_cfg)) {
        uint32_t vec = IGB_IVAR_ENTRY_VEC(int_cfg);
        if (vec < IGB_MSIX_VEC_NUM) {
            if (!e1000e_eitr_should_postpone(core, vec)) {
                trace_e1000e_irq_msix_notify_vec(vec);
                msix_notify(core->owner, vec);
            }
        } else {
            trace_e1000e_wrn_msix_vec_wrong(cause, int_cfg);
        }
    } else {
        trace_e1000e_wrn_msix_invalid(cause, int_cfg);
    }

    if (core->mac[GPIE] & E1000_GPIE_EIAME) {
        trace_e1000e_irq_iam_clear_eiame(core->mac[EIAM], cause);
        core->mac[EIAM] &= ~cause;
    }

    trace_e1000e_irq_icr_clear_eiac(core->mac[EICR], core->mac[EIAC]);

    effective_eiac = core->mac[EIAC] & cause;

    core->mac[EICR] &= ~effective_eiac;

    if (!(core->mac[GPIE] & E1000_GPIE_IAME)) {
        core->mac[EIMS] &= ~effective_eiac;
    }
}

static void
e1000e_msix_notify(E1000ECore *core, uint32_t causes)
{
    uint16_t index = 0;
    uint32_t cause = 0x1;
    uint32_t causes_left = causes;
    while (causes_left) {
        if (cause) {
            igb_msix_notify_one(core, cause,
                                igb_get_ivar_entry(core, ));
    }

    if (causes & E1000_ICR_RXQ1) {
        e1000e_msix_notify_one(core, E1000_ICR_RXQ1,
                               E1000_IVAR_RXQ1(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_TXQ0) {
        e1000e_msix_notify_one(core, E1000_ICR_TXQ0,
                               E1000_IVAR_TXQ0(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_TXQ1) {
        e1000e_msix_notify_one(core, E1000_ICR_TXQ1,
                               E1000_IVAR_TXQ1(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_OTHER) {
        e1000e_msix_notify_one(core, E1000_ICR_OTHER,
                               E1000_IVAR_OTHER(core->mac[IVAR]));
    }
}

static void
e1000e_msix_clear_one(E1000ECore *core, uint32_t cause, uint32_t int_cfg)
{
    if (E1000_IVAR_ENTRY_VALID(int_cfg)) {
        uint32_t vec = E1000_IVAR_ENTRY_VEC(int_cfg);
        if (vec < E1000E_MSIX_VEC_NUM) {
            trace_e1000e_irq_msix_pending_clearing(cause, int_cfg, vec);
            msix_clr_pending(core->owner, vec);
        } else {
            trace_e1000e_wrn_msix_vec_wrong(cause, int_cfg);
        }
    } else {
        trace_e1000e_wrn_msix_invalid(cause, int_cfg);
    }
}

static void
e1000e_msix_clear(E1000ECore *core, uint32_t causes)
{
    if (causes & E1000_ICR_RXQ0) {
        e1000e_msix_clear_one(core, E1000_ICR_RXQ0,
                              E1000_IVAR_RXQ0(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_RXQ1) {
        e1000e_msix_clear_one(core, E1000_ICR_RXQ1,
                              E1000_IVAR_RXQ1(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_TXQ0) {
        e1000e_msix_clear_one(core, E1000_ICR_TXQ0,
                              E1000_IVAR_TXQ0(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_TXQ1) {
        e1000e_msix_clear_one(core, E1000_ICR_TXQ1,
                              E1000_IVAR_TXQ1(core->mac[IVAR]));
    }

    if (causes & E1000_ICR_OTHER) {
        e1000e_msix_clear_one(core, E1000_ICR_OTHER,
                              E1000_IVAR_OTHER(core->mac[IVAR]));
    }
}

static inline void
e1000e_fix_icr_asserted(E1000ECore *core)
{
    core->mac[ICR] &= ~E1000_ICR_ASSERTED;
    if (core->mac[ICR]) {
        core->mac[ICR] |= E1000_ICR_ASSERTED;
    }

    trace_e1000e_irq_fix_icr_asserted(core->mac[ICR]);
}

static void
e1000e_send_msi(E1000ECore *core, bool msix)
{
    uint32_t causes = core->mac[ICR] & core->mac[IMS] & ~E1000_ICR_ASSERTED;

    if (msix) {
        e1000e_msix_notify(core, causes);
    } else {
        if (!e1000e_itr_should_postpone(core)) {
            trace_e1000e_irq_msi_notify(causes);
            msi_notify(core->owner, 0);
        }
    }
}

static void
e1000e_update_interrupt_state(E1000ECore *core)
{
    bool interrupts_pending;
    bool is_msix = msix_enabled(core->owner);

    /* Set ICR[OTHER] for MSI-X */
    if (is_msix) {
        if (core->mac[ICR] & E1000_ICR_OTHER_CAUSES) {
            core->mac[ICR] |= E1000_ICR_OTHER;
            trace_e1000e_irq_add_msi_other(core->mac[ICR]);
        }
    }

    e1000e_fix_icr_asserted(core);

    /*
     * Make sure ICR and ICS registers have the same value.
     * The spec says that the ICS register is write-only.  However in practice,
     * on real hardware ICS is readable, and for reads it has the same value as
     * ICR (except that ICS does not have the clear on read behaviour of ICR).
     *
     * The VxWorks PRO/1000 driver uses this behaviour.
     */
    core->mac[ICS] = core->mac[ICR];

    interrupts_pending = (core->mac[IMS] & core->mac[ICR]) ? true : false;

    trace_e1000e_irq_pending_interrupts(core->mac[ICR] & core->mac[IMS],
                                        core->mac[ICR], core->mac[IMS]);

    if (is_msix || msi_enabled(core->owner)) {
        if (interrupts_pending) {
            e1000e_send_msi(core, is_msix);
        }
    } else {
        if (interrupts_pending) {
            if (!e1000e_itr_should_postpone(core)) {
                e1000e_raise_legacy_irq(core);
            }
        } else {
            e1000e_lower_legacy_irq(core);
        }
    }
}

static void
e1000e_set_interrupt_cause(E1000ECore *core, uint32_t val)
{
    trace_e1000e_irq_set_cause_entry(val, core->mac[ICR]);

    val |= e1000e_intmgr_collect_delayed_causes(core);
    core->mac[ICR] |= val;

    trace_e1000e_irq_set_cause_exit(val, core->mac[ICR]);

    e1000e_update_interrupt_state(core);
}
#endif
