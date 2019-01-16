/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(GD_GPL)
 */

#ifndef __KERNEL_BOOT_H
#define __KERNEL_BOOT_H

#include <bootinfo.h>

#ifndef CONFIG_ARCH_ARM
#define MAX_NUM_FREEMEM_REG 16
#else
#define MAX_NUM_FREEMEM_REG (ARRAY_SIZE(avail_p_regs) + 1)
#endif /* CONFIG_ARCH_ARM */

/* Resolve naming differences between the abstract specifications
 * of the bootstrapping phase and the runtime phase of the kernel. */
typedef cte_t  slot_t;
typedef cte_t* slot_ptr_t;
#define SLOT_PTR(pptr, pos) (((slot_ptr_t)(pptr)) + (pos))
#define pptr_of_cap (pptr_t)cap_get_capPtr

/* (node-local) state accessed only during bootstrapping.
 *
 * Acts as a temporary store until bootstrapping is complete and the data can
 * be moved into the boot info frame for the initial thread. */
typedef struct ndks_boot {
    cte_t root_cnode;
    seL4_SlotPos next_root_cnode_slot;

    region_t freemem[MAX_NUM_FREEMEM_REG];

    /* Auto-generated code for managing untyped capabilities makes assumptions
     * about alignment. This is not automatically enforced by the compiler so
     * annotation is required.
     *
     * Note that as the boot code has been reworked to retype untyped
     * capabilities into kernel objects, this array will be exceeded earlier
     * than previously anticipated. */
    cte_t untyped[CONFIG_MAX_NUM_BOOTINFO_UNTYPED_CAPS]
            __attribute__((aligned (8)));
    seL4_SlotPos next_untyped_slot;
    seL4_SlotRegion root_untyped_slots;

    seL4_BootInfo *bi_frame;
    seL4_SlotPos slot_pos_cur;
    seL4_SlotPos slot_pos_max;
} ndks_boot_t;

extern ndks_boot_t ndks_boot;

/* function prototypes */

static inline bool_t
is_reg_empty(region_t reg)
{
    return reg.start == reg.end;
}

void init_ndks(void);
void init_empty_cslot(cte_t *cslot);
seL4_SlotRegion create_untypeds_for_region(region_t reg);
void create_root_untypeds(void);
bool_t alloc_kernel_object(cte_t *dest, object_t type, word_t user_size_bits);
bool_t create_root_cnode(void);
void provide_cslot_to_root_cnode(cte_t *src, word_t offset);
void create_domain_cap(void);
bool_t create_irq_cnode(void);
cte_t *get_cslot_from_root_cnode(word_t offset);
bool_t create_it_asid_pool(void);
bool_t allocate_bi_frame(vptr_t vptr, node_id_t node_id, word_t num_nodes,
        vptr_t ipcbuf_vptr);
bool_t create_ipcbuf_frame(vptr_t vptr);
bool_t create_ui_frames(region_t ui_reg, sword_t pv_offset);

pptr_t alloc_region(word_t size_bits);
bool_t insert_region(region_t reg);
void write_slot(slot_ptr_t slot_ptr, cap_t cap);
bool_t provide_cap(cap_t root_cnode_cap, cap_t cap);
void write_it_pd_pts(cap_t root_cnode_cap, cap_t it_pd_cap);
bool_t create_idle_thread(void);
void bi_finalise(void);

region_t allocate_extra_bi_region(word_t extra_size);

cap_t
create_it_pd_pts(
    cap_t      root_cnode_cap,
    v_region_t ui_v_reg,
    vptr_t     ipcbuf_vptr,
    vptr_t     bi_frame_vptr
);

tcb_t *
create_initial_thread(
    cap_t  root_cnode_cap,
    cap_t  it_pd_cap,
    vptr_t ui_v_entry,
    vptr_t bi_frame_vptr,
    vptr_t ipcbuf_vptr,
    cap_t  ipcbuf_cap
);

void init_core_state(tcb_t *scheduler_action);
#endif /* __KERNEL_BOOT_H */
