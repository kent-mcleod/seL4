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

BOOT_CODE pptr_t
alloc_region(word_t size_bits)
{
    word_t i;
    word_t reg_index = 0; /* gcc cannot work out that this will not be used uninitialized */
    region_t reg = REG_EMPTY;
    region_t rem_small = REG_EMPTY;
    region_t rem_large = REG_EMPTY;
    region_t new_reg;
    region_t new_rem_small;
    region_t new_rem_large;

    /* Search for a freemem region that will be the best fit for an allocation. We favour allocations
     * that are aligned to either end of the region. If an allocation must split a region we favour
     * an unbalanced split. In both cases we attempt to use the smallest region possible. In general
     * this means we aim to make the size of the smallest remaining region smaller (ideally zero)
     * followed by making the size of the largest remaining region smaller */

    for (i = 0; i < MAX_NUM_FREEMEM_REG; i++) {
        /* Determine whether placing the region at the start or the end will create a bigger left over region */
        if (ROUND_UP(ndks_boot.freemem[i].start, size_bits) - ndks_boot.freemem[i].start <
                ndks_boot.freemem[i].end - ROUND_DOWN(ndks_boot.freemem[i].end, size_bits)) {
            new_reg.start = ROUND_UP(ndks_boot.freemem[i].start, size_bits);
            new_reg.end = new_reg.start + BIT(size_bits);
        } else {
            new_reg.end = ROUND_DOWN(ndks_boot.freemem[i].end, size_bits);
            new_reg.start = new_reg.end - BIT(size_bits);
        }
        if (new_reg.end > new_reg.start &&
                new_reg.start >= ndks_boot.freemem[i].start &&
                new_reg.end <= ndks_boot.freemem[i].end) {
            if (new_reg.start - ndks_boot.freemem[i].start < ndks_boot.freemem[i].end - new_reg.end) {
                new_rem_small.start = ndks_boot.freemem[i].start;
                new_rem_small.end = new_reg.start;
                new_rem_large.start = new_reg.end;
                new_rem_large.end = ndks_boot.freemem[i].end;
            } else {
                new_rem_large.start = ndks_boot.freemem[i].start;
                new_rem_large.end = new_reg.start;
                new_rem_small.start = new_reg.end;
                new_rem_small.end = ndks_boot.freemem[i].end;
            }
            if ( is_reg_empty(reg) ||
                    (reg_size(new_rem_small) < reg_size(rem_small)) ||
                    (reg_size(new_rem_small) == reg_size(rem_small) && reg_size(new_rem_large) < reg_size(rem_large)) ) {
                reg = new_reg;
                rem_small = new_rem_small;
                rem_large = new_rem_large;
                reg_index = i;
            }
        }
    }
    if (is_reg_empty(reg)) {
        printf("Kernel init failing: not enough memory\n");
        return 0;
    }
    /* Remove the region in question */
    ndks_boot.freemem[reg_index] = REG_EMPTY;
    /* Add the remaining regions in largest to smallest order */
    insert_region(rem_large);
    if (!insert_region(rem_small)) {
        printf("alloc_region(): wasted 0x%lx bytes due to alignment, try to increase MAX_NUM_FREEMEM_REG\n",
               (word_t)(rem_small.end - rem_small.start));
    }
    return reg.start;
}

BOOT_CODE void
write_slot(slot_ptr_t slot_ptr, cap_t cap)
{
    slot_ptr->cap = cap;

    slot_ptr->cteMDBNode = nullMDBNode;
    mdb_node_ptr_set_mdbRevocable  (&slot_ptr->cteMDBNode, true);
    mdb_node_ptr_set_mdbFirstBadged(&slot_ptr->cteMDBNode, true);
}

BOOT_CODE cap_t
create_ipcbuf_frame(cap_t root_cnode_cap, cap_t pd_cap, vptr_t vptr)
{
    cap_t cap;
    pptr_t pptr;

    /* allocate the IPC buffer frame */
    pptr = alloc_region(PAGE_BITS);
    if (!pptr) {
        printf("Kernel init failing: could not create ipc buffer frame\n");
        return cap_null_cap_new();
    }
    clearMemory((void*)pptr, PAGE_BITS);

    /* create a cap of it and write it into the root CNode */
    cap = create_mapped_it_frame_cap(pd_cap, pptr, vptr, IT_ASID, false, false);
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadIPCBuffer), cap);

    return cap;
}

BOOT_CODE void
create_bi_frame_cap(
    cap_t      root_cnode_cap,
    cap_t      pd_cap,
    pptr_t     pptr,
    vptr_t     vptr
)
{
    cap_t cap;

    /* create a cap of it and write it into the root CNode */
    cap = create_mapped_it_frame_cap(pd_cap, pptr, vptr, IT_ASID, false, false);
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapBootInfoFrame), cap);
}

BOOT_CODE region_t
allocate_extra_bi_region(word_t extra_size)
{
    /* determine power of 2 size of this region. avoid calling clzl on 0 though */
    if (extra_size == 0) {
        /* return any valid address to correspond to the zero allocation */
        return (region_t) {
            0x1000, 0x1000
        };
    }
    word_t size_bits = seL4_WordBits - 1 - clzl(ROUND_UP(extra_size, seL4_PageBits));
    pptr_t pptr = alloc_region(size_bits);
    if (!pptr) {
        printf("Kernel init failed: could not allocate extra bootinfo region size bits %lu\n", size_bits);
        return REG_EMPTY;
    }

    clearMemory((void*)pptr, size_bits);
    ndks_boot.bi_frame->extraLen = BIT(size_bits);

    return (region_t) {
        pptr, pptr + BIT(size_bits)
    };
}

BOOT_CODE pptr_t
allocate_bi_frame(
    node_id_t  node_id,
    word_t   num_nodes,
    vptr_t ipcbuf_vptr
)
{
    pptr_t pptr;

    /* create the bootinfo frame object */
    pptr = alloc_region(BI_FRAME_SIZE_BITS);
    if (!pptr) {
        printf("Kernel init failed: could not allocate bootinfo frame\n");
        return 0;
    }
    clearMemory((void*)pptr, BI_FRAME_SIZE_BITS);

    /* initialise bootinfo-related global state */
    ndks_boot.bi_frame = BI_PTR(pptr);
    ndks_boot.slot_pos_cur = seL4_NumInitialCaps;

    BI_PTR(pptr)->nodeID = node_id;
    BI_PTR(pptr)->numNodes = num_nodes;
    BI_PTR(pptr)->numIOPTLevels = 0;
    BI_PTR(pptr)->ipcBuffer = (seL4_IPCBuffer *) ipcbuf_vptr;
    BI_PTR(pptr)->initThreadCNodeSizeBits = CONFIG_ROOT_CNODE_SIZE_BITS;
    BI_PTR(pptr)->initThreadDomain = ksDomSchedule[ksDomScheduleIdx].domain;
    BI_PTR(pptr)->extraLen = 0;
    BI_PTR(pptr)->extraBIPages.start = 0;
    BI_PTR(pptr)->extraBIPages.end = 0;

    return pptr;
}

BOOT_CODE bool_t
provide_cap(cap_t root_cnode_cap, cap_t cap)
{
    if (ndks_boot.slot_pos_cur >= ndks_boot.slot_pos_max) {
        printf("Kernel init failed: ran out of cap slots\n");
        return false;
    }
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), ndks_boot.slot_pos_cur), cap);
    ndks_boot.slot_pos_cur++;
    return true;
}

BOOT_CODE create_frames_of_region_ret_t
create_frames_of_region(
    cap_t    root_cnode_cap,
    cap_t    pd_cap,
    region_t reg,
    bool_t   do_map,
    sword_t  pv_offset
)
{
    pptr_t     f;
    cap_t      frame_cap;
    seL4_SlotPos slot_pos_before;
    seL4_SlotPos slot_pos_after;

    slot_pos_before = ndks_boot.slot_pos_cur;

    for (f = reg.start; f < reg.end; f += BIT(PAGE_BITS)) {
        if (do_map) {
            frame_cap = create_mapped_it_frame_cap(pd_cap, f, pptr_to_paddr((void*)(f - pv_offset)), IT_ASID, false, true);
        } else {
            frame_cap = create_unmapped_it_frame_cap(f, false);
        }
        if (!provide_cap(root_cnode_cap, frame_cap))
            return (create_frames_of_region_ret_t) {
            S_REG_EMPTY, false
        };
    }

    slot_pos_after = ndks_boot.slot_pos_cur;

    return (create_frames_of_region_ret_t) {
        (seL4_SlotRegion) { slot_pos_before, slot_pos_after }, true
    };
}

BOOT_CODE cap_t
create_it_asid_pool(cap_t root_cnode_cap)
{
    pptr_t ap_pptr;
    cap_t  ap_cap;

    /* create ASID pool */
    ap_pptr = alloc_region(seL4_ASIDPoolBits);
    if (!ap_pptr) {
        printf("Kernel init failed: failed to create initial thread asid pool\n");
        return cap_null_cap_new();
    }
    memzero(ASID_POOL_PTR(ap_pptr), 1 << seL4_ASIDPoolBits);
    ap_cap = cap_asid_pool_cap_new(IT_ASID >> asidLowBits, ap_pptr);
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadASIDPool), ap_cap);

    /* create ASID control cap */
    write_slot(
        SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapASIDControl),
        cap_asid_control_cap_new()
    );

    return ap_cap;
}

BOOT_CODE bool_t
create_idle_thread(void)
{
    pptr_t pptr;

#ifdef ENABLE_SMP_SUPPORT
    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
#endif /* ENABLE_SMP_SUPPORT */
        pptr = alloc_region(seL4_TCBBits);
        if (!pptr) {
            printf("Kernel init failed: Unable to allocate tcb for idle thread\n");
            return false;
        }
        memzero((void *)pptr, 1 << seL4_TCBBits);
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

BOOT_CODE tcb_t *
create_initial_thread(
    cap_t  root_cnode_cap,
    cap_t  it_pd_cap,
    vptr_t ui_v_entry,
    vptr_t bi_frame_vptr,
    vptr_t ipcbuf_vptr,
    cap_t  ipcbuf_cap
)
{
    pptr_t pptr;
    cap_t  cap;
    tcb_t* tcb;
    deriveCap_ret_t dc_ret;

    /* allocate TCB */
    pptr = alloc_region(seL4_TCBBits);
    if (!pptr) {
        printf("Kernel init failed: Unable to allocate tcb for initial thread\n");
        return NULL;
    }
    memzero((void*)pptr, 1 << seL4_TCBBits);
    tcb = TCB_PTR(pptr + TCB_OFFSET);
    tcb->tcbTimeSlice = CONFIG_TIME_SLICE;
    Arch_initContext(&tcb->tcbArch.tcbContext);

    /* derive a copy of the IPC buffer cap for inserting */
    dc_ret = deriveCap(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadIPCBuffer), ipcbuf_cap);
    if (dc_ret.status != EXCEPTION_NONE) {
        printf("Failed to derive copy of IPC Buffer\n");
        return NULL;
    }

    /* initialise TCB (corresponds directly to abstract specification) */
    cteInsert(
        root_cnode_cap,
        SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadCNode),
        SLOT_PTR(pptr, tcbCTable)
    );
    cteInsert(
        it_pd_cap,
        SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadVSpace),
        SLOT_PTR(pptr, tcbVTable)
    );
    cteInsert(
        dc_ret.cap,
        SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadIPCBuffer),
        SLOT_PTR(pptr, tcbBuffer)
    );
    tcb->tcbIPCBuffer = ipcbuf_vptr;

    /* Set the root thread's IPC buffer */
    Arch_setTCBIPCBuffer(tcb, ipcbuf_vptr);

    setRegister(tcb, capRegister, bi_frame_vptr);
    setNextPC(tcb, ui_v_entry);

    /* initialise TCB */
    tcb->tcbPriority = seL4_MaxPrio;
    tcb->tcbMCP = seL4_MaxPrio;
    setupReplyMaster(tcb);
    setThreadState(tcb, ThreadState_Running);

    ksCurDomain = ksDomSchedule[ksDomScheduleIdx].domain;
    ksDomainTime = ksDomSchedule[ksDomScheduleIdx].length;
    assert(ksCurDomain < CONFIG_NUM_DOMAINS && ksDomainTime > 0);

    SMP_COND_STATEMENT(tcb->tcbAffinity = 0);

    /* create initial thread's TCB cap */
    cap = cap_thread_cap_new(TCB_REF(tcb));
    write_slot(SLOT_PTR(pptr_of_cap(root_cnode_cap), seL4_CapInitThreadTCB), cap);

#ifdef CONFIG_DEBUG_BUILD
    setThreadName(tcb, "rootserver");
#endif

    return tcb;
}

BOOT_CODE void
init_core_state(tcb_t *scheduler_action)
{
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
    seL4_SlotPos slot_pos_start = ndks_boot.slot_pos_cur;
    seL4_SlotPos slot_pos_end = ndks_boot.slot_pos_max;
    ndks_boot.bi_frame->empty = (seL4_SlotRegion) {
        slot_pos_start, slot_pos_end
    };
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

    /* The root cnode must be provided to itself. This would lose the
     * in ndks_boot. So that this reference can be used when creating
     * other kernel objects, the root cnode will be provided when
     * finishing the bootstrapping of the kernel. */
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
