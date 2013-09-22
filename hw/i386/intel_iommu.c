/*
 * QEMU emulation of an Intel IOMMU 
 *   (DMA Remapping device)
 *
 * Copyright (c) 2013 Knut Omang, Oracle <knut.omang@oracle.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "hw/i386/intel_iommu.h"

#define DEBUG_INTEL_IOMMU
#ifdef DEBUG_INTEL_IOMMU
#define D(fmt, ...) \
    do { fprintf(stderr, "%s, %s: " fmt "\n", __FILE__, __func__, ## __VA_ARGS__); } while (0)
#else
#define D(fmt, ...) \
    do { } while (0)
#endif


static inline void define_quad(intel_iommu_state *s, hwaddr addr, uint64_t val, 
                        uint64_t wmask, uint64_t w1cmask)
{
    *((uint64_t*)&s->csr[addr]) = val;
    *((uint64_t*)&s->wmask[addr]) = wmask;
    *((uint64_t*)&s->w1cmask[addr]) = w1cmask;
}

static inline void define_long(intel_iommu_state *s, hwaddr addr, uint32_t val, 
                        uint32_t wmask, uint32_t w1cmask)
{
    *((uint32_t*)&s->csr[addr]) = val;
    *((uint32_t*)&s->wmask[addr]) = wmask;
    *((uint32_t*)&s->w1cmask[addr]) = w1cmask;
}

static inline void define_long_wo(intel_iommu_state *s, hwaddr addr, uint32_t mask)
{
    *((uint32_t*)&s->womask[addr]) = mask;
}

/* "External" set-operations */
static inline void set_quad(intel_iommu_state *s, hwaddr addr, uint64_t val)
{
    uint64_t oldval = *((uint64_t*)&s->csr[addr]);
    uint64_t wmask = *((uint64_t*)&s->wmask[addr]);
    uint64_t w1cmask = *((uint64_t*)&s->w1cmask[addr]);
    *((uint64_t*)&s->csr[addr]) = 
        ((oldval & ~wmask) | (val & wmask)) & ~(w1cmask & val);
}

static inline void set_long(intel_iommu_state *s, hwaddr addr, uint32_t val)
{
    uint32_t oldval = *((uint32_t*)&s->csr[addr]);
    uint32_t wmask = *((uint32_t*)&s->wmask[addr]);
    uint32_t w1cmask = *((uint32_t*)&s->w1cmask[addr]);
    *((uint32_t*)&s->csr[addr]) = 
        ((oldval & ~wmask) | (val & wmask)) & ~(w1cmask & val);
}

static inline uint64_t get_quad(intel_iommu_state *s, hwaddr addr)
{
    uint64_t val = *((uint64_t*)&s->csr[addr]);
    uint64_t womask = *((uint64_t*)&s->womask[addr]);
    return val & ~womask;
}

static inline uint32_t get_long(intel_iommu_state *s, hwaddr addr)
{
    uint32_t val = *((uint32_t*)&s->csr[addr]);
    uint32_t womask = *((uint32_t*)&s->womask[addr]);
    return val & ~womask;
}

static inline uint32_t set_mask_long(intel_iommu_state *s, hwaddr addr, uint32_t mask)
{
    uint32_t oldval = *((uint32_t*)&s->csr[addr]);
    uint32_t val = oldval | mask;
    *((uint32_t*)&s->csr[addr]) = val;
    return val;
}

static void iommu_inv_queue_setup(intel_iommu_state *s) 
{
    uint64_t tail_val;
    s->iq = *((uint64_t*)&s->csr[DMAR_IQA_REG]);
    s->iq_sz = 0x100 << (s->iq & 0x7);  /* 256 entries per page */
    s->iq &= ~0x7;
    s->iq_enable = true;

    /* Init head pointers */
    tail_val = *((uint64_t*)&s->csr[DMAR_IQT_REG]);
    *((uint64_t*)&s->csr[DMAR_IQH_REG]) = tail_val;
    s->iq_head = s->iq_tail = (tail_val >> 4) & 0x7fff;
    D(" -- address: 0x%lx size 0x%lx", s->iq, s->iq_sz);
}


static void handle_gcmd_qie(intel_iommu_state *s, bool en)
{
    D("Queued Invalidation Enable %s", (en ? "on" : "off"));

    if (en)
        iommu_inv_queue_setup(s);

    /* Ok - report back to driver */
    set_mask_long(s, DMAR_GSTS_REG, DMA_GSTS_QIES);
}


static void iommu_root_table_setup(intel_iommu_state *s) 
{
    s->root = *((uint64_t*)&s->csr[DMAR_RTADDR_REG]);
    s->extended = (s->root >> 11) & 1;
    s->root &= ~0xfff;
    D(" -- address: 0x%lx %s", s->root, (s->extended ? "(Extended)" : "")); 
}


#define status_write(x) (((x) >> 5) & 1)


static int handle_invalidate(intel_iommu_state *s, uint16_t i)
{
    intel_iommu_inv_desc entry;
    uint8_t type;
    dma_memory_read(&address_space_memory, s->iq + sizeof(entry) * i, &entry, sizeof(entry));
    type = entry.lower & 0xf;
    D(" Processing invalidate request %d - desc: %016lx.%016lx", i, entry.upper, entry.lower);    
    switch (type) {
    case CONTEXT_CACHE_INV_DESC:
        D("Context-cache Invalidate");
        break;
    case IOTLB_INV_DESC:
        D("IOTLB Invalidate");
        break;
    case INV_WAIT_DESC:
        D("Invalidate Wait");
        if (status_write(entry.lower))
            dma_memory_write(&address_space_memory, entry.upper, (uint8_t*)&entry.lower + 4, 4);
        break;
    default:
        D(" - not impl - ");
    }
    return 0;
}


static void handle_iqt_write(intel_iommu_state *s, uint64_t val)
{
    s->iq_tail = (val >> 4) & 0x7fff;
    D("Write to IQT_REG new tail = %d", s->iq_tail);

    if (!s->iq_enable) return;

    /* Process the invalidation queue */
    while (s->iq_head != s->iq_tail) {
        handle_invalidate(s, s->iq_head++);
        if (s->iq_head == s->iq_sz)
            s->iq_head = 0;
    }
    *((uint64_t*)&s->csr[DMAR_IQH_REG]) = s->iq_head << 4;
}


static void handle_gcmd_srtp(intel_iommu_state *s, bool en)
{
    D("Set Root Table Pointer %s", (en ? "on" : "off"));
    
    if (en) 
        iommu_root_table_setup(s);

    /* Ok - report back to driver */
    set_mask_long(s, DMAR_GSTS_REG, DMA_GSTS_RTPS);
}


static void handle_gcmd_te(intel_iommu_state *s, bool en)
{
    D("Translation Enable %s", (en ? "on" : "off"));

    

    /* Ok - report back to driver */
    set_mask_long(s, DMAR_GSTS_REG, DMA_GSTS_TES);
}


static void handle_gcmd_write(intel_iommu_state *s, uint32_t val)
{
    uint32_t oldval = *((uint32_t*)&s->csr[DMAR_GCMD_REG]);
    uint32_t changed = oldval ^ val;
    if (changed & DMA_GCMD_TE) {
        handle_gcmd_te(s, val & DMA_GCMD_TE);
    }
    if (val & DMA_GCMD_SRTP) {
        handle_gcmd_srtp(s, val & DMA_GCMD_SRTP);
    }
    if (changed & DMA_GCMD_SFL) {
        D("Set Fault Log %s", (val & DMA_GCMD_SFL ? "on" : "off"));
    }
    if (changed & DMA_GCMD_EAFL) {
        D("Enable Advanced Fault Logging %s", (val & DMA_GCMD_EAFL ? "on" : "off"));
    }
    if (changed & DMA_GCMD_WBF) {
        D("Write Buffer Flush %s", (val & DMA_GCMD_WBF ? "on" : "off"));
    }
    if (changed & DMA_GCMD_QIE) {
        handle_gcmd_qie(s, val & DMA_GCMD_QIE);
    }
    if (changed & DMA_GCMD_SIRTP) {
        D("Interrupt Remapping Enable %s", (val & DMA_GCMD_SIRTP ? "on" : "off"));
    }
    if (changed & DMA_GCMD_IRE) {
        D("Set Interrupt Remapping Table Pointer %s", (val & DMA_GCMD_IRE ? "on" : "off"));
    }
    if (changed & DMA_GCMD_CFI) {
        D("Compatibility Format Interrupt %s", (val & DMA_GCMD_CFI ? "on" : "off"));
    }
}


static uint64_t intel_iommu_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    intel_iommu_state *s = opaque;
    uint64_t val;
    if (addr + size > DMAR_REG_SIZE) {
        D("addr outside region: max %lx, got %lx %x ", addr + size, addr, size);
        return (uint64_t)-1;
    }

    if (size == 4)
        val = get_long(s, addr);
    else if (size == 8)
        val = get_quad(s, addr);
    else
        val = (uint64_t)-1;

    D(" addr %lx size %d val %lx", addr, size, val);
    return val;
}

static void intel_iommu_mem_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    intel_iommu_state *s = opaque;
    if (addr + size > DMAR_REG_SIZE) {
        D("addr outside region: max %lx, got %lx %x ", addr + size, addr, size);
        return;
    }
    D(" addr %lx size %d val %lx", addr, size, val);
    s->csr[addr] = val;
    if (size == 4) {
        switch (addr) {
        case DMAR_GCMD_REG:
            handle_gcmd_write(s, val);
            break;
        case DMAR_IQT_REG:
            handle_iqt_write(s, val);
            break;
        }
        set_long(s, addr, val);
    } else if (size == 8) {
        switch (addr) {
        case DMAR_IQT_REG:
            handle_iqt_write(s, val);
            break;
        }
        set_quad(s, addr, val);
    }
}

static IOMMUTLBEntry intel_iommu_translate(MemoryRegion *iommu, hwaddr addr)
{
    return (IOMMUTLBEntry) {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = TARGET_PAGE_MASK,
        .perm = IOMMU_RW,
    };
}

static const VMStateDescription intel_iommu_vmstate = {
    .name ="iommu_intel",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField []) {
        VMSTATE_UINT8_ARRAY(csr, intel_iommu_state, DMAR_REG_SIZE),
        VMSTATE_END_OF_LIST()
    }
};


static const MemoryRegionOps intel_iommu_mem_ops = {
    .read = intel_iommu_mem_read,
    .write = intel_iommu_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static MemoryRegionIOMMUOps intel_iommu_ops = {
    .translate = intel_iommu_translate,
};


static Property iommu_properties[] = {
    DEFINE_PROP_HEX32("version", intel_iommu_state, version, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void intel_iommu_reset(DeviceState *d)
{
    /* intel_iommu_state *s = INTEL_IOMMU_DEVICE(d); */
    D(" ");
}

static int intel_iommu_init(SysBusDevice *dev)
{
    intel_iommu_state *s = INTEL_IOMMU_DEVICE(dev);
    memory_region_init_io(&s->csrmem, OBJECT(s), &intel_iommu_mem_ops, s, "intel_iommu",
                          DMAR_REG_SIZE);
    D(" ");

    /* b.0:2 = 2: Number of domains supported: 256 using 8 bit ids 
     * b.3   = 0: No advanced fault logging
     * b.4   = 0: No required write buffer flushing
     * b.5   = 1: Protected low memory region supported
     * b.6   = 1: Protected high memory region supported
     */
    const uint64_t dmar_cap_reg_value = 0xc0000020e60262L;

    /* Define registers with default values and bit semantics */
    define_quad(s, DMAR_VER_REG, 0x10, 0, 0);  /* set MAX = 1, RO */
    define_quad(s, DMAR_CAP_REG, dmar_cap_reg_value, 0, 0);
    define_quad(s, DMAR_ECAP_REG, 0xf0101aL, 0, 0);
    define_long(s, DMAR_GCMD_REG, 0, 0xffffffff, 0); 
    define_long_wo(s, DMAR_GCMD_REG, 0xffc00000);
    define_long(s, DMAR_GSTS_REG, 0, 0, 0); /* All bits RO, default 0 */
    define_quad(s, DMAR_RTADDR_REG, 0, 0xfffffffffffff000L, 0);
    define_quad(s, DMAR_CCMD_REG, 0x800000000000000L, 0xe00000030000ffffL, 0);
    define_long(s, DMAR_FSTS_REG, 0, 0, 0xfd);
    define_long(s, DMAR_FECTL_REG, 0x80000000, 0x80000000, 0);
    define_long(s, DMAR_FEDATA_REG, 0, 0xffffffff, 0); /* All bits RW */
    define_long(s, DMAR_FEADDR_REG, 0, 0xfffffffc, 0); /* 31:2 RW */
    define_long(s, DMAR_FEUADDR_REG, 0, 0xffffffff, 0); /* 31:2 RW */

    define_quad(s, DMAR_AFLOG_REG, 0, 0xffffffffffffff00L, 0);
    define_long(s, DMAR_PMEN_REG, 0, 0x80000000, 0);
    /* TBD: The definition of these are dynamic:
     * DMAR_PLMBASE_REG, DMAR_PLMLIMIT_REG, DMAR_PHMBASE_REG, DMAR_PHMLIMIT_REG
     */
    define_quad(s, DMAR_IQH_REG, 0, 0, 0);  /* Bits 18:4 (0x3fff0) is RO, rest is RsvdZ */
    define_quad(s, DMAR_IQT_REG, 0, 0x3fff0, 0);
    define_quad(s, DMAR_IQA_REG, 0, 0xfffffffffffff000L, 0);
    define_quad(s, DMAR_ICS_REG, 0, 0, 0x1); /* Bit 0 is RW1CS - rest is RsvdZ */
    define_long(s, DMAR_IECTL_REG, 0x80000000, 0x80000000, 0); /* b.31 is RW, b.30 RO, rest: RsvdZ */
    define_long(s, DMAR_IEDATA_REG, 0, 0xffffffff, 0);
    define_long(s, DMAR_IEADDR_REG, 0, 0xfffffffc, 0);
    define_long(s, DMAR_IEUADDR_REG, 0, 0xffffffff, 0);
    define_quad(s, DMAR_IRTA_REG, 0, 0xfffffffffffff80fL, 0);
    define_quad(s, DMAR_PQH_REG, 0, 0x3fff0L, 0);
    define_quad(s, DMAR_PQT_REG, 0, 0x3fff0L, 0);
    define_quad(s, DMAR_PQA_REG, 0, 0xfffffffffffff007L, 0);
    define_long(s, DMAR_PRS_REG, 0, 0, 0x1);
    define_long(s, DMAR_PECTL_REG, 0, 0x80000000, 0);
    define_long(s, DMAR_PEDATA_REG, 0, 0xffffffff, 0);
    define_long(s, DMAR_PEADDR_REG, 0, 0xfffffffc, 0);
    define_long(s, DMAR_PEUADDR_REG, 0, 0xffffffff, 0);
    return 0;
}

static void iommu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = intel_iommu_init;
    dc->reset = intel_iommu_reset;
    dc->vmsd = &intel_iommu_vmstate;
    dc->props = iommu_properties;
}

static const TypeInfo iommu_info = {
    .name          = TYPE_INTEL_IOMMU_DEVICE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(intel_iommu_state),
    .class_init    = iommu_class_init,
};

static void iommu_register_types(void)
{
    D(" ");
    type_register_static(&iommu_info);
}

type_init(iommu_register_types)
