/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <config.h>

#ifdef CONFIG_ARM_SMMU
#include <arch/object/smmu.h>

exception_t decodeARMSIDControlInvocation(word_t label, unsigned int length, cptr_t cptr,
	cte_t *srcSlot, cap_t cap, extra_caps_t extraCaps,
	bool_t call, word_t *buffer) {

	word_t index, depth, sid;
	cte_t *destSlot;
	cap_t cnodeCap;
	lookupSlot_ret_t lu_ret;
	exception_t status;

	if (label != ARMSIDIssueSIDManager) {
		userError("SIDControl: Illegal operation.");
		current_syscall_error.type = seL4_IllegalOperation;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (length < 3 || extraCaps.excaprefs[0] == NULL) {
		current_syscall_error.type = seL4_TruncatedMessage;
		return EXCEPTION_SYSCALL_ERROR;
	}

	sid = getSyscallArg(0, buffer);
	index = getSyscallArg(1, buffer);
	depth = getSyscallArg(2, buffer);
	cnodeCap = extraCaps.excaprefs[0]->cap;

    if (sid >= SMMU_MAX_SID) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = SMMU_MAX_SID - 1;
        userError("Rejecting request for SID %u. SID is greater than or equal to SMMU_MAX_SID.", (int)sid);
        return EXCEPTION_SYSCALL_ERROR;
    }
	if (smmuStateSIDTable[sid]) {
		current_syscall_error.type = seL4_RevokeFirst;
		userError("Rejecting request for SID %u. Already active.", (int)sid);
		return EXCEPTION_SYSCALL_ERROR;
	}

	lu_ret = lookupTargetSlot(cnodeCap, index, depth);
	if (lu_ret.status != EXCEPTION_NONE) {
		userError("Target slot for new SID Handler cap invalid: cap %lu, SID %u.",
			getExtraCPtr(buffer, 0), (int)sid);
		return lu_ret.status;
	}
	destSlot = lu_ret.slot;
	status = ensureEmptySlot(destSlot);
	if (status != EXCEPTION_NONE) {
		userError("Target slot for new SID Handler cap not empty: cap %lu, SID %u.",
			getExtraCPtr(buffer, 0), (int)sid);
		return status;
	}
	setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
	smmuStateSIDTable[sid] = true;
    cteInsert(cap_sid_cap_new(0, sid), srcSlot, destSlot);
	return EXCEPTION_NONE;
}

exception_t decodeARMSIDInvocation(word_t label, unsigned int length, cptr_t cptr,
	cte_t *srcSlot, cap_t cap, extra_caps_t extraCaps,
	bool_t call, word_t *buffer) {
	cap_t cbCap;
	cte_t *cbCapSlot;
	cte_t *cbAssignSlot;
	exception_t status;
	word_t sid;

	if (unlikely(label != ARMSIDBindCB)) {
		userError("ARMSID: Illegal operation.");
		current_syscall_error.type = seL4_IllegalOperation;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (unlikely(extraCaps.excaprefs[0] == NULL)) {
		userError("ARMSIDBindCB: Invalid CB cap.");
		current_syscall_error.type = seL4_TruncatedMessage;
		return EXCEPTION_SYSCALL_ERROR;
	}

	cbCapSlot = extraCaps.excaprefs[0];
	cbCap = cbCapSlot->cap;

	if (unlikely(cap_get_capType(cbCap) != cap_cb_cap)) {
		userError("ARMSIDBindCB: Invalid CB cap.");
		current_syscall_error.type = seL4_InvalidCapability;
		current_syscall_error.invalidCapNumber = 1;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (unlikely(!cap_cb_cap_get_capCBIsMapped(cbCap))) {
		userError("ARMSIDBindCB: Invalid CB cap.");
		current_syscall_error.type = seL4_InvalidCapability;
		current_syscall_error.invalidCapNumber = 1;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (unlikely(cap_sid_cap_get_capSIDIsMapped(cap))) {
		userError("ARMSIDBindCB: The SID is already bound with a context bank.");
		current_syscall_error.type = seL4_RevokeFirst;
		return EXCEPTION_SYSCALL_ERROR;
	}

	/*the SID number is valid as created by ARMSIDIssueSIDManager*/
	sid = cap_sid_cap_get_capSID(cap);
	cbAssignSlot = smmuStateSIDNode + sid;
	status = ensureEmptySlot(cbAssignSlot);
	if (status != EXCEPTION_NONE) {
		userError("ARMSIDBindCB: The SID is already bound with a context bank.");
		return status;
	}
	cap_sid_cap_ptr_set_capSIDIsMapped(&(srcSlot->cap), 1);
	/*binding the sid to cb in SMMU*/
	smmu_sid_bind_cb(sid, cap_cb_cap_get_capCB(cbCap));
	setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
	/* Building the connection between SID and CB caps by placing a
	 * copy of the given cb cap in sid's cnode*/
	cteInsert(cbCap, cbCapSlot, cbAssignSlot);
	/* Recording the SID number in the copied CB cap.
	 * Deleting the copied CB cap will trigger unbinding
	 * operations.  As a CB can be used (bound)
	 * by multiple SID caps, each copied CB caps resulted from
	 * binding operations keeps track of its serving SID numbers.*/
	cap_cb_cap_ptr_set_capBindSID(&(cbAssignSlot->cap), sid);
	return EXCEPTION_NONE;
}

exception_t decodeARMCBControlInvocation(word_t label, unsigned int length, cptr_t cptr,
	cte_t *srcSlot, cap_t cap, extra_caps_t extraCaps,
	bool_t call, word_t *buffer) {

	word_t index, depth, cb;
	cte_t *destSlot;
	cap_t cnodeCap;
	lookupSlot_ret_t lu_ret;
	exception_t status;

	if (label != ARMCBIssueCBManager) {
		userError("ARMCBControl: Illegal operation.");
		current_syscall_error.type = seL4_IllegalOperation;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (length < 3 || extraCaps.excaprefs[0] == NULL) {
		current_syscall_error.type = seL4_TruncatedMessage;
		return EXCEPTION_SYSCALL_ERROR;
	}

	cb = getSyscallArg(0, buffer);
	index = getSyscallArg(1, buffer);
	depth = getSyscallArg(2, buffer);
	cnodeCap = extraCaps.excaprefs[0]->cap;

    if (cb >= SMMU_MAX_CB) {
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 0;
        current_syscall_error.rangeErrorMax = SMMU_MAX_CB - 1;
        userError("Rejecting request for CB %u. CB is greater than or equal to SMMU_MAX_CB.", (int)cb);
        return EXCEPTION_SYSCALL_ERROR;
    }
	if (smmuStateCBTable[cb]) {
		current_syscall_error.type = seL4_RevokeFirst;
		userError("Rejecting request for CB %u. Already active.", (int)cb);
		return EXCEPTION_SYSCALL_ERROR;
	}

	lu_ret = lookupTargetSlot(cnodeCap, index, depth);
	if (lu_ret.status != EXCEPTION_NONE) {
		userError("Target slot for new SID Handler cap invalid: cap %lu, CB %u.",
			getExtraCPtr(buffer, 0), (int)cb);
		return lu_ret.status;
	}
	destSlot = lu_ret.slot;
	status = ensureEmptySlot(destSlot);
	if (status != EXCEPTION_NONE) {
		userError("Target slot for new CB Handler cap not empty: cap %lu, CB %u.",
			getExtraCPtr(buffer, 0), (int)cb);
		return status;
	}

	setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
	smmuStateCBTable[cb] = true;
	cteInsert(cap_cb_cap_new(0, SID_INVALID, cb), srcSlot, destSlot);
	return EXCEPTION_NONE;
}

exception_t decodeARMCBInvocation(word_t label, unsigned int length, cptr_t cptr,
	cte_t *srcSlot, cap_t cap, extra_caps_t extraCaps,
	bool_t call, word_t *buffer) {

	cap_t vspaceCap;
	cte_t *vspaceCapSlot;
	cte_t *cbSlot;
	exception_t status;
	word_t cb; 

	if (unlikely(label != ARMCBAssignVspace)) {
		userError("ARMCBAssignVspace: Illegal operation.");
		current_syscall_error.type = seL4_IllegalOperation;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (unlikely(extraCaps.excaprefs[0] == NULL)) {
		current_syscall_error.type = seL4_TruncatedMessage;
		return EXCEPTION_SYSCALL_ERROR;
	}

	vspaceCapSlot = extraCaps.excaprefs[0];
	vspaceCap = vspaceCapSlot->cap;


	if (unlikely(!isVTableRoot(vspaceCap) || !cap_vtable_root_isMapped(vspaceCap))) {
		userError("ARMCBAssignVspace: the vspace is invalid");
		current_syscall_error.type = seL4_InvalidCapability;
		current_syscall_error.invalidCapNumber = 1;
		return EXCEPTION_SYSCALL_ERROR;
	}
	if (unlikely(cap_cb_cap_get_capCBIsMapped(cap))) {
		userError("ARMCBAssignVspace: the CB already assigned with a vspace root.");
		current_syscall_error.type = seL4_RevokeFirst;
		return EXCEPTION_SYSCALL_ERROR;
	}
	/*the cb number must be valid as it is created via the ARMCBIssueCBManager*/
	cb = cap_cb_cap_get_capCB(cap); 
	cbSlot = smmuStateCBNode + cb;
	status = ensureEmptySlot(cbSlot);
	if (status != EXCEPTION_NONE) {
		userError("ARMCBAssignVspace: the CB already assigned with a vspace root.");
		return status;
	}
	cap_cb_cap_ptr_set_capCBIsMapped(&(srcSlot->cap), 1); 
	/*setting up cb in smmu*/
	smmu_cb_assign_vspace(cb, cap_vtable_root_get_basePtr(vspaceCap), 
		cap_vtable_root_get_mappedASID(vspaceCap)); 
	setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
	/*building connection between the vspace cap and  cb*/
	cteInsert(vspaceCap, vspaceCapSlot, cbSlot);
	cap_vtable_root_ptr_set_mappedCB(&(cbSlot->cap), cb); 
	return EXCEPTION_NONE;
}
#endif

