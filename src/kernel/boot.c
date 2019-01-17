/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(GD_GPL)
 */

#include <assert.h>
#include <kernel/boot.h>
#include <kernel/thread.h>
#include <machine/io.h>
#include <machine/registerset.h>
#include <model/statedata.h>
#include <arch/machine.h>
#include <arch/kernel/boot.h>
#include <arch/kernel/vspace.h>
#include <linker.h>
#include <plat/machine/hardware.h>
#include <util.h>
#include <object/untyped.h>
#include <object/objecttype.h>

/* (node-local) state accessed only during bootstrapping */
#define IRQ_CNODE_BITS (seL4_WordBits - clzl(maxIRQ * sizeof(cte_t)))

ndks_boot_t ndks_boot BOOT_DATA;

BOOT_CODE bool_t
insert_region(region_t reg)
{
    word_t i;

    assert(reg.start <= reg.end);
    if (is_reg_empty(reg)) {
        return true;
    }
    for (i = 0; i < MAX_NUM_FREEMEM_REG; i++) {
        if (is_reg_empty(ndks_boot.freemem[i])) {
            ndks_boot.freemem[i] = reg;
            return true;
        }
    }
    return false;
}

BOOT_CODE static inline word_t
reg_size(region_t reg)
{
    return reg.end - reg.start;
}

BOOT_CODE void
init_ndks(void)
{
    /* Ensures that the ndks struct has a sane initial state. */
    ndks_boot.next_root_cnode_slot = seL4_NumInitialCaps;
    ndks_boot.next_untyped_slot = 0;
}

BOOT_CODE static inline bool_t
is_untyped_slot_available(void)
{
    return ndks_boot.next_untyped_slot < CONFIG_MAX_NUM_BOOTINFO_UNTYPED_CAPS;
}

BOOT_CODE void
init_empty_cslot(cte_t *cslot)
{
    /* Dangerous - use cautiously.
     *
     * Use of this function is required to ensure a sane initial state before
     * performing operations. For instance, stale stack data can cause bugs. */
    cslot->cap = cap_null_cap_new();
    cslot->cteMDBNode = nullMDBNode;
}

BOOT_CODE static cte_t *
alloc_untyped_slot(void)
{
    cte_t *ut_slot;

    if (!is_untyped_slot_available()) {
        return NULL;
    }

    ut_slot = &ndks_boot.untyped[ndks_boot.next_untyped_slot];
    ndks_boot.next_untyped_slot++;

    /* Guarantee sane initial state. */
    init_empty_cslot(ut_slot);
    return ut_slot;
}

BOOT_CODE seL4_SlotRegion
create_untypeds_for_region(region_t reg)
{

    cte_t *ut_slot;
    word_t align_bits, size_bits;
    seL4_SlotRegion created_untypeds;

    created_untypeds.start = ndks_boot.next_untyped_slot;

    while (!is_reg_empty(reg)) {
        /* Determine the maximum size of the region. */
        size_bits = seL4_WordBits - 1 - clzl(reg_size(reg));

        /* Determine the alignment of the region. */
        if (reg.start != 0) {
            align_bits = ctzl(reg.start);
        } else {
            align_bits = size_bits;
        }

        /* Reduce size_bits for alignment if needed. */
        if (align_bits < size_bits) {
            size_bits = align_bits;
        }
        if (size_bits > seL4_MaxUntypedBits) {
            size_bits = seL4_MaxUntypedBits;
        }

        if (size_bits >= seL4_MinUntypedBits) {
            ut_slot = alloc_untyped_slot();
            if (ut_slot == NULL) {
                /* Allocation can fail when the number of temporary cslots
                 * required exceeds the available number. */
                break;
            }

            assert(ensureEmptySlot(ut_slot) == EXCEPTION_NONE);

            /* Creating untyped cap to represent this portion of memory. */
            ut_slot->cap = cap_untyped_cap_new(MAX_FREE_INDEX(size_bits), false,
                    size_bits, reg.start);

            /* Initialising a sane state for the MDB node. */
            ut_slot->cteMDBNode = nullMDBNode;
            mdb_node_ptr_set_mdbRevocable(&ut_slot->cteMDBNode, true);
            mdb_node_ptr_set_mdbFirstBadged(&ut_slot->cteMDBNode, true);
        }

        reg.start += BIT(size_bits);
    }

    created_untypeds.end = ndks_boot.next_untyped_slot;
    return created_untypeds;
}

BOOT_CODE void
create_root_untypeds(void)
{
    word_t i;
    seL4_SlotRegion used_untyped_slots;

    /* By definition, root untypeds must be created before any other
     * untypeds. */
    assert(ndks_boot.next_untyped_slot == 0);

    for (i = 0; i < MAX_NUM_FREEMEM_REG; i++) {
        if (!is_untyped_slot_available()) {
            /* This check exists in create_untypeds_for_region but is also
             * here so that this loop can early exit. */
            break;
        }

        used_untyped_slots = create_untypeds_for_region(ndks_boot.freemem[i]);
        ndks_boot.freemem[i] = REG_EMPTY;
    }

    /* Marking the untypeds used as root untypeds. */
    ndks_boot.root_untyped_slots = (seL4_SlotRegion) {
        0,
        used_untyped_slots.end
    };
}

BOOT_CODE static inline word_t
align_up(word_t base_value, word_t alignment)
{
    return (base_value + (BIT(alignment) - 1)) & ~MASK(alignment);
}

BOOT_CODE static bool_t
untyped_retype_wrapper(cte_t *src, cte_t *dest, object_t type,
        word_t user_size_bits)
{
    /* This function is a BOOT_CODE wrapper around invokeUntyped_Retype and
     * maintains some of the semantics that invokeUntyped_Retype has.
     *
     * Note that user_size_bits is not necessarily the actual size in memory.
     * This is explained in a comment in invokeUntyped_Retype. */
    bool_t reset, device_memory;
    word_t object_size_bits, untyped_free_bytes;
    word_t free_index, free_ref;
    exception_t status;

    /* The wrapper attempts to make some of the sanity checks that are present
     * in decodeUntypedInvocation but they are not complete. As the wrapper is
     * only used internally, it requires the caller to provide correct
     * arguments. */
    assert(cap_get_capType(src->cap) == cap_untyped_cap);
    assert(ensureEmptySlot(dest) == EXCEPTION_NONE);
    assert(type < seL4_ObjectTypeCount);

    device_memory = cap_untyped_cap_get_capIsDevice(src->cap);
    assert(!device_memory || Arch_isFrameType(type));

    object_size_bits = getObjectSize(type, user_size_bits);
    assert(user_size_bits < wordBits);
    assert(object_size_bits <= seL4_MaxUntypedBits);

    assert(type != seL4_CapTableObject || user_size_bits > 0);
    assert(type != seL4_UntypedObject || user_size_bits >= seL4_MinUntypedBits);

    status = ensureNoChildren(src);
    if (status != EXCEPTION_NONE) {
        free_index = cap_untyped_cap_get_capFreeIndex(src->cap);
        reset = false;
    } else {
        free_index = 0;
        reset = true;
    }

    /* There may not be enough room to retype the requested object. */
    free_ref = GET_FREE_REF(cap_untyped_cap_get_capPtr(src->cap), free_index);
    untyped_free_bytes = BIT(cap_untyped_cap_get_capBlockSize(src->cap)) -
                         FREE_INDEX_TO_OFFSET(free_index);
    if (untyped_free_bytes < BIT(object_size_bits)) {
        /* Function is only expected to fail when not enough space
         * is available in the untyped. */
        return false;
    }

    free_ref = align_up(free_ref, object_size_bits);
    status = invokeUntyped_Retype(
        src,
        reset,
        (void *) free_ref,
        type,
        object_size_bits,
        (slot_range_t) {
            dest,
            0,
            1
        },
        device_memory);
    assert(status == EXCEPTION_NONE);

    return true;
}

BOOT_CODE bool_t
alloc_kernel_object(cte_t *dest, object_t type, word_t user_size_bits)
{
    /* Kernel objects are allocated so that when revoked, the underlying
     * memory can be reused.
     *
     * Instead of retyping a root untyped immediately into a kernel object, it
     * is first retyped into an untyped the same size as the kernel object.
     * This is done so that individual kernel objects can be revoked and
     * memory reused without requiring all kernel objects allocated during
     * bootstrapping to be revoked.
     *
     * This function can be used to create an untyped of a specific size. In
     * this case, a second retype is obviously not necessary and so not
     * done.
     *
     * Note that user_size_bits is not necessarily the actual size of the
     * object in memory. See untyped_retype_wrapper for details. */
    bool_t status;
    word_t i, object_size_bits;
    cte_t *root_ut, *child_ut;

    assert(ensureEmptySlot(dest) == EXCEPTION_NONE);

    if (type == seL4_UntypedObject) {
        child_ut = dest;
    } else {
        child_ut = alloc_untyped_slot();
        if (child_ut == NULL) {
            return false;
        }
    }

    /* Because of the special treatment of the size of some kernel object
     * types, the untyped object must be created with the expected object
     * size. */
    object_size_bits = getObjectSize(type, user_size_bits);

    /* Iterating through each of the root untypeds and attempting to
     * retype them into the required size. */
    for (i = 0; i < ndks_boot.root_untyped_slots.end; i++) {
        root_ut = &ndks_boot.untyped[i];
        status = untyped_retype_wrapper(root_ut, child_ut, seL4_UntypedObject,
                object_size_bits);
        if (status) {
            break;
        }
    }

    if (!status) {
        /* No root untyped with enough space for the requested object can be
         * found. */
        return false;
    }

    if (type != seL4_UntypedObject) {
        status = untyped_retype_wrapper(child_ut, dest, type, user_size_bits);
        assert(status);
    }

    return true;
}

/* Our root CNode needs to be able to fit all the initial caps and not
 * cover all of memory. */
compile_assert(root_cnode_size_valid,
               CONFIG_ROOT_CNODE_SIZE_BITS < 32 - seL4_SlotBits &&
               (1U << CONFIG_ROOT_CNODE_SIZE_BITS) >= seL4_NumInitialCaps)

BOOT_CODE bool_t
create_root_cnode(void)
{
    bool_t status;
    word_t user_size_bits;

    user_size_bits = CONFIG_ROOT_CNODE_SIZE_BITS;
    status = alloc_kernel_object(&ndks_boot.root_cnode, seL4_CapTableObject,
            user_size_bits);
    if (!status) {
        return false;
    }

    /* When retyping into a seL4_CapTableObject the badge size is different
     * to the expected badge size for the root cnode. */
    cap_cnode_cap_ptr_set_capCNodeGuardSize(&ndks_boot.root_cnode.cap,
            wordBits - CONFIG_ROOT_CNODE_SIZE_BITS);

    /* The root cnode must be provided to itself. This would lose the
     * in ndks_boot. So that this reference can be used when creating
     * other kernel objects, the root cnode will be provided when
     * finishing the bootstrapping of the kernel. */
    return true;
}

BOOT_CODE void
provide_cslot_to_root_cnode(cte_t *src, word_t offset)
{
    cte_t *cnode, *dest;

    assert(offset < BIT(CONFIG_ROOT_CNODE_SIZE_BITS));

    cnode = &ndks_boot.root_cnode;
    assert(ensureEmptySlot(cnode) != EXCEPTION_NONE);

    dest = SLOT_PTR(pptr_of_cap(cnode->cap), offset);
    assert(ensureEmptySlot(dest) == EXCEPTION_NONE);

    cteMove(src->cap, src, dest);
}

compile_assert(num_domains_valid,
               CONFIG_NUM_DOMAINS >= 1 && CONFIG_NUM_DOMAINS <= 256)
compile_assert(num_priorities_valid,
               CONFIG_NUM_PRIORITIES >= 1 && CONFIG_NUM_PRIORITIES <= 256)

BOOT_CODE void
create_domain_cap(void)
{
    /* Should never fail. */
    word_t i;
    cte_t domain;

    /* Check domain scheduler assumptions. */
    assert(ksDomScheduleLength > 0);
    for (i = 0; i < ksDomScheduleLength; i++) {
        assert(ksDomSchedule[i].domain < CONFIG_NUM_DOMAINS);
        assert(ksDomSchedule[i].length > 0);
    }

    domain.cap = cap_domain_cap_new();
    domain.cteMDBNode = nullMDBNode;
    mdb_node_ptr_set_mdbRevocable(&domain.cteMDBNode, true);
    mdb_node_ptr_set_mdbFirstBadged(&domain.cteMDBNode, true);

    provide_cslot_to_root_cnode(&domain, seL4_CapDomain);
}

BOOT_CODE bool_t
create_irq_cnode(void)
{
    cte_t irq_cnode;
    bool_t status;
    pptr_t pptr;

    init_empty_cslot(&irq_cnode);
    status = alloc_kernel_object(&irq_cnode, seL4_CapTableObject, IRQ_CNODE_BITS);
    if (!status) {
        return false;
    }

    pptr = pptr_of_cap(irq_cnode.cap);
    intStateIRQNode = (cte_t *) pptr;

    return true;
}

BOOT_CODE cte_t *
get_cslot_from_root_cnode(word_t offset)
{
    pptr_t pptr;

    assert(offset < BIT(CONFIG_ROOT_CNODE_SIZE_BITS));

    pptr = pptr_of_cap(ndks_boot.root_cnode.cap);
    return SLOT_PTR(pptr, offset);
}

BOOT_CODE bool_t
create_it_asid_pool(void)
{
    bool_t status;

    cte_t *ut;
    cte_t asid_pool;
    cte_t asid_control;

    pptr_t pptr;

    ut = alloc_untyped_slot();
    if (ut == NULL) {
        return false;
    }

    status = alloc_kernel_object(ut, seL4_UntypedObject, seL4_ASIDPoolBits);
    if (!status) {
        return false;
    }

    /* Performing manual untyped retype.
     * This is done manually because invokeUntyped_Retype cannot retype into
     * an asid pool. */
    init_empty_cslot(&asid_pool);
    pptr = pptr_of_cap(ut->cap);

    cteInsert(cap_asid_pool_cap_new(IT_ASID >> asidLowBits, WORD_REF(pptr)),
            ut, &asid_pool);
    provide_cslot_to_root_cnode(&asid_pool, seL4_CapInitThreadASIDPool);

    asid_control.cap = cap_asid_control_cap_new();
    asid_control.cteMDBNode = nullMDBNode;
    mdb_node_ptr_set_mdbRevocable(&asid_control.cteMDBNode, true);
    mdb_node_ptr_set_mdbFirstBadged(&asid_control.cteMDBNode, true);

    provide_cslot_to_root_cnode(&asid_control, seL4_CapASIDControl);
    return true;
}

BOOT_CODE bool_t
allocate_bi_frame(vptr_t vptr, node_id_t node_id, word_t num_nodes,
        vptr_t ipcbuf_vptr)
{
    bool_t status;
    cte_t frame;

    seL4_BootInfo *bi_frame;

    status = create_it_frame(&frame);
    if (!status) {
        return false;
    }

    map_it_frame(&frame, vptr, false);

    ndks_boot.bi_frame = BI_PTR(pptr_of_cap(frame.cap));
    provide_cslot_to_root_cnode(&frame, seL4_CapBootInfoFrame);

    bi_frame = ndks_boot.bi_frame;
    bi_frame->nodeID = node_id;
    bi_frame->numNodes = num_nodes;
    bi_frame->numIOPTLevels = 0;
    bi_frame->ipcBuffer = (seL4_IPCBuffer *) ipcbuf_vptr;
    bi_frame->initThreadCNodeSizeBits = CONFIG_ROOT_CNODE_SIZE_BITS;
    bi_frame->initThreadDomain = ksDomSchedule[ksDomScheduleIdx].domain;
    bi_frame->extraLen = 0;
    bi_frame->extraBIPages.start = 0;
    bi_frame->extraBIPages.end = 0;

    return true;
}

BOOT_CODE bool_t
create_ipcbuf_frame(vptr_t vptr)
{
    bool_t status;
    cte_t frame;

    status = create_it_frame(&frame);
    if (!status) {
        return false;
    }

    map_it_frame(&frame, vptr, false);
    return true;
}

bool_t create_ui_frames(region_t ui_reg, sword_t pv_offset)
{
    word_t i;

    cte_t frame;
    seL4_SlotRegion ui_frames; /* Stored in root cnode. */

    ui_frames.start = ndks_boot.next_root_cnode_slot;
    for (i = ui_reg.start; i < ui_reg.end; i += BIT(PAGE_BITS)) {

        /* TODO: Provide untypeds for frames.
         * Currently not done to avoid having to solve the problem of where
         * they are stored. */
        frame.cap = create_unmapped_it_frame_cap(i, false);
        frame.cteMDBNode = nullMDBNode;
        mdb_node_ptr_set_mdbRevocable(&frame.cteMDBNode, true);
        mdb_node_ptr_set_mdbFirstBadged(&frame.cteMDBNode, true);

        map_it_frame(&frame, pptr_to_paddr((void *)(i - pv_offset)), true);

        provide_cslot_to_root_cnode(&frame, ndks_boot.next_root_cnode_slot);
        ndks_boot.next_root_cnode_slot++;
    }

    ui_frames.end = ndks_boot.next_root_cnode_slot;
    ndks_boot.bi_frame->userImageFrames = ui_frames;
    return true;
}

BOOT_CODE bool_t
create_idle_thread(void)
{
    cte_t tcb;
    pptr_t pptr;
    bool_t status;

#ifdef ENABLE_SMP_SUPPORT
    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
#endif /* ENABLE_SMP_SUPPORT */
        init_empty_cslot(&tcb);

        status = alloc_kernel_object(&tcb, seL4_TCBObject, seL4_TCBBits);
        if (!status) {
            return false;
        }

        pptr = pptr_of_cap(tcb.cap);

        NODE_STATE_ON_CORE(ksIdleThread, i) = TCB_PTR(pptr + TCB_OFFSET);
        configureIdleThread(NODE_STATE_ON_CORE(ksIdleThread, i));
#ifdef CONFIG_DEBUG_BUILD
        setThreadName(NODE_STATE_ON_CORE(ksIdleThread, i), "idle_thread");
#endif
        SMP_COND_STATEMENT(NODE_STATE_ON_CORE(ksIdleThread, i)->tcbAffinity = i);
#ifdef ENABLE_SMP_SUPPORT
    }
#endif /* ENABLE_SMP_SUPPORT */
    return true;
}

BOOT_CODE bool_t
create_initial_thread(vptr_t ui_v_entry, vptr_t bi_frame_vptr,
        vptr_t ipcbuf_vptr)
{
    deriveCap_ret_t dc_ret;

    tcb_t *tcb;
    cte_t tcb_cslot;
    pptr_t tcb_pptr;

    bool_t status;

    cte_t *ipc_buf;
    cte_t *root_cnode;
    cte_t *pd;
    cte_t *ipc;

    init_empty_cslot(&tcb_cslot);
    status = alloc_kernel_object(&tcb_cslot, seL4_TCBObject, seL4_TCBBits);
    if (!status) {
        return false;
    }

    tcb_pptr = pptr_of_cap(tcb_cslot.cap);
    tcb = TCB_PTR(tcb_pptr + TCB_OFFSET);
    tcb->tcbTimeSlice = CONFIG_TIME_SLICE;
    Arch_initContext(&tcb->tcbArch.tcbContext);

    /* Derive a copy of the IPC buffer cap for inserting. */
    ipc_buf = get_cslot_from_root_cnode(seL4_CapInitThreadIPCBuffer);
    dc_ret = deriveCap(ipc_buf, ipc_buf->cap);
    if (dc_ret.status != EXCEPTION_NONE) {
        printf("Failed to derive copy of IPC Buffer\n");
        return false;
    }

    /* Initialise TCB (corresponds directly to abstract specification). */
    root_cnode = &ndks_boot.root_cnode;
    cteInsert(
        root_cnode->cap,
        root_cnode,
        SLOT_PTR(tcb_pptr, tcbCTable)
    );

    pd = get_cslot_from_root_cnode(seL4_CapInitThreadVSpace);
    cteInsert(
        pd->cap,
        pd,
        SLOT_PTR(tcb_pptr, tcbVTable)
    );

    ipc = get_cslot_from_root_cnode(seL4_CapInitThreadIPCBuffer);
    cteInsert(
        dc_ret.cap,
        ipc,
        SLOT_PTR(tcb_pptr, tcbBuffer)
    );
    tcb->tcbIPCBuffer = ipcbuf_vptr;

    /* Set the root thread's IPC buffer. */
    Arch_setTCBIPCBuffer(tcb, ipcbuf_vptr);

    setRegister(tcb, capRegister, bi_frame_vptr);
    setNextPC(tcb, ui_v_entry);

    /* Initialise TCB. */
    tcb->tcbPriority = seL4_MaxPrio;
    tcb->tcbMCP = seL4_MaxPrio;
    setupReplyMaster(tcb);
    setThreadState(tcb, ThreadState_Running);

    ksCurDomain = ksDomSchedule[ksDomScheduleIdx].domain;
    ksDomainTime = ksDomSchedule[ksDomScheduleIdx].length;
    assert(ksCurDomain < CONFIG_NUM_DOMAINS && ksDomainTime > 0);

    SMP_COND_STATEMENT(tcb->tcbAffinity = 0);

    provide_cslot_to_root_cnode(&tcb_cslot, seL4_CapInitThreadTCB);

#ifdef CONFIG_DEBUG_BUILD
    setThreadName(tcb, "rootserver");
#endif

    return true;
}

BOOT_CODE void
init_core_state(void)
{
    cte_t *tcb_cslot;
    pptr_t tcb_pptr;
    tcb_t *scheduler_action;

    tcb_cslot = get_cslot_from_root_cnode(seL4_CapInitThreadTCB);
    assert(ensureEmptySlot(tcb_cslot) != EXCEPTION_NONE);

    tcb_pptr = pptr_of_cap(tcb_cslot->cap);
    scheduler_action = TCB_PTR(tcb_pptr + TCB_OFFSET);

#ifdef CONFIG_HAVE_FPU
    NODE_STATE(ksActiveFPUState) = NULL;
#endif
#ifdef CONFIG_DEBUG_BUILD
    /* add initial threads to the debug queue */
    NODE_STATE(ksDebugTCBs) = NULL;
    if (scheduler_action != SchedulerAction_ResumeCurrentThread &&
            scheduler_action != SchedulerAction_ChooseNewThread) {
        tcbDebugAppend(scheduler_action);
    }
    tcbDebugAppend(NODE_STATE(ksIdleThread));
#endif
    NODE_STATE(ksSchedulerAction) = scheduler_action;
    NODE_STATE(ksCurThread) = NODE_STATE(ksIdleThread);
}

BOOT_CODE void
bi_finalise(void)
{
    /* Untyped capabilities must be sorted and inserted into the root cnode.
     * Metadata about the untypeds must then be insterted into the boot info
     * frame.
     *
     * TODO: Provide the used untypeds to the boot info frame.
     * This isn't done yet because its not necessary for a successful boot. */
    word_t i;
    pptr_t pptr;

    cte_t *root_ut;
    seL4_SlotRegion root_ut_region;

    root_ut_region.start = ndks_boot.next_root_cnode_slot;

    for (i = 0; i < ndks_boot.root_untyped_slots.end; i++) {
        root_ut = &ndks_boot.untyped[i];

        /* Adding metadata for the untyped to the boot info frame. */
        pptr = pptr_of_cap(root_ut->cap);
        ndks_boot.bi_frame->untypedList[i] = (seL4_UntypedDesc) {
            pptr_to_paddr((void *) pptr),
            0, /* Padding. */
            0, /* Padding. */
            cap_get_capSizeBits(root_ut->cap),
            false
        };

        /* Inserting the root untyped into the root cnode. */
        provide_cslot_to_root_cnode(root_ut, ndks_boot.next_root_cnode_slot);
        ndks_boot.next_root_cnode_slot++;
    }

    root_ut_region.end = ndks_boot.next_root_cnode_slot;
    ndks_boot.bi_frame->untyped = root_ut_region;
    ndks_boot.bi_frame->userImagePaging = ndks_boot.user_paging_slots;
    ndks_boot.bi_frame->empty = (seL4_SlotRegion) {
        ndks_boot.next_root_cnode_slot,
        BIT(CONFIG_ROOT_CNODE_SIZE_BITS)
    };
}
