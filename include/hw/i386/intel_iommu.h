/*
 * QEMU emulation of an Intel IOMMU 
 *   (DMA Remapping device)
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
 *
 * Lots of defines copied from kernel/include/linux/intel-iommu.h:
 *   Copyright (C) 2006-2008 Intel Corporation
 *   Author: Ashok Raj <ashok.raj@intel.com>
 *   Author: Anil S Keshavamurthy <anil.s.keshavamurthy@intel.com>
 *
 * Copyright (c) 2013 Knut Omang, Oracle <knut.omang@oracle.com>
 *
 */

#ifndef _INTEL_IOMMU_H
#define _INTEL_IOMMU_H
#include "hw/qdev.h"
#include "sysemu/dma.h"

#define TYPE_INTEL_IOMMU_DEVICE "intel-iommu"
#define INTEL_IOMMU_DEVICE(obj) \
     OBJECT_CHECK(intel_iommu_state, (obj), TYPE_INTEL_IOMMU_DEVICE)

/* DMAR Hardware Unit Definition address (IOMMU unit) set i bios */
#define Q35_HOST_BRIDGE_IOMMU_ADDR 0xfed90000

/*
 * Intel IOMMU register specification per version 1.0 public spec.
 */

#define	DMAR_VER_REG	0x0	/* Arch version supported by this IOMMU */
#define	DMAR_CAP_REG	0x8	/* Hardware supported capabilities */
#define	DMAR_ECAP_REG	0x10	/* Extended capabilities supported */
#define	DMAR_GCMD_REG	0x18	/* Global command register */
#define	DMAR_GSTS_REG	0x1c	/* Global status register */
#define	DMAR_RTADDR_REG	0x20	/* Root entry table */
#define	DMAR_CCMD_REG	0x28	/* Context command reg */
#define	DMAR_FSTS_REG	0x34	/* Fault Status register */
#define	DMAR_FECTL_REG	0x38	/* Fault control register */
#define	DMAR_FEDATA_REG	0x3c	/* Fault event interrupt data register */
#define	DMAR_FEADDR_REG	0x40	/* Fault event interrupt addr register */
#define	DMAR_FEUADDR_REG 0x44	/* Upper address register */
#define	DMAR_AFLOG_REG	0x58	/* Advanced Fault control */
#define	DMAR_PMEN_REG	0x64	/* Enable Protected Memory Region */
#define	DMAR_PLMBASE_REG 0x68	/* PMRR Low addr */
#define	DMAR_PLMLIMIT_REG 0x6c	/* PMRR low limit */
#define	DMAR_PHMBASE_REG 0x70	/* pmrr high base addr */
#define	DMAR_PHMLIMIT_REG 0x78	/* pmrr high limit */
#define DMAR_IQH_REG	0x80	/* Invalidation queue head register */
#define DMAR_IQT_REG	0x88	/* Invalidation queue tail register */
#define DMAR_IQ_SHIFT	4	/* Invalidation queue head/tail shift */
#define DMAR_IQA_REG	0x90	/* Invalidation queue addr register */
#define DMAR_ICS_REG	0x98	/* Invalidation complete status register */
#define DMAR_IRTA_REG	0xb8    /* Interrupt remapping table addr register */

/* From Vt-d 2.2 spec */
#define DMAR_IECTL_REG  0xa0    /* Invalidation event control register */
#define DMAR_IEDATA_REG 0xa4    /* Invalidation event data register */
#define DMAR_IEADDR_REG 0xa8    /* Invalidation event address register */
#define DMAR_IEUADDR_REG 0xac    /* Invalidation event address register */
#define DMAR_PQH_REG    0xc0    /* Page request queue head register */
#define DMAR_PQT_REG    0xc8    /* Page request queue tail register*/
#define DMAR_PQA_REG    0xd0    /* Page request queue address register */
#define DMAR_PRS_REG    0xd8    /* Page request status register */
#define DMAR_PECTL_REG  0xe0    /* Page request event control register */
#define DMAR_PEDATA_REG 0xe4    /* Page request event data register */
#define DMAR_PEADDR_REG 0xe8    /* Page request event address register */
#define DMAR_PEUADDR_REG  0xec  /* Page event upper address register */
#define DMAR_MTRRCAP_REG 0x100  /* MTRR capability register */
#define DMAR_MTRRDEF_REG 0x108  /* MTRR default type register */


#define DMAR_REG_SIZE   0x220    /* Last reg 64 bit at 0x218 */

/* The iommu (DMAR) device state struct */

typedef struct intel_iommu_state {
    SysBusDevice busdev;
    MemoryRegion csrmem;
    MemoryRegion iommu;
    uint8_t csr[DMAR_REG_SIZE];     /* register values */
    uint8_t wmask[DMAR_REG_SIZE];   /* R/W bytes */
    uint8_t w1cmask[DMAR_REG_SIZE]; /* RW1C(Write 1 to Clear) bytes */
    uint8_t womask[DMAR_REG_SIZE]; /* WO (write only - read returns 0) */
    uint32_t version;

    dma_addr_t root;  /* Current root table pointer */
    bool extended;    /* Type of root table (extended or not) */
    uint16_t iq_head; /* Current invalidation queue tail */
    uint16_t iq_tail; /* Current invalidation queue tail */
    dma_addr_t iq;   /* Current invalidation queue (IQ) pointer */
    size_t iq_sz;    /* IQ Size in number of entries */
    bool iq_enable;  /* Set if the IQ is enabled */
} intel_iommu_state;


/* An invalidate descriptor */
typedef struct intel_iommu_inv_desc
{
    uint64_t lower;
    uint64_t upper;
} intel_iommu_inv_desc;


/* Invalidate descriptor types */
#define CONTEXT_CACHE_INV_DESC  0x1
#define PASID_CACHE_INV_DESC    0x7
#define IOTLB_INV_DESC          0x2
#define EXT_IOTLB_INV_DESC      0x6
#define DEV_TLB_INV_DESC        0x3
#define EXT_DEV_TLB_INV_DESC    0x8
#define INT_ENTRY_INV_DESC      0x4
#define INV_WAIT_DESC           0x5


/* IOTLB_REG */
#define DMA_TLB_FLUSH_GRANU_OFFSET  60
#define DMA_TLB_GLOBAL_FLUSH (((uint64_t)1) << 60)
#define DMA_TLB_DSI_FLUSH (((uint64_t)2) << 60)
#define DMA_TLB_PSI_FLUSH (((uint64_t)3) << 60)
#define DMA_TLB_IIRG(type) ((type >> 60) & 7)
#define DMA_TLB_IAIG(val) (((val) >> 57) & 7)
#define DMA_TLB_READ_DRAIN (((uint64_t)1) << 49)
#define DMA_TLB_WRITE_DRAIN (((uint64_t)1) << 48)
#define DMA_TLB_DID(id)	(((uint64_t)((id) & 0xffff)) << 32)
#define DMA_TLB_IVT (((uint64_t)1) << 63)
#define DMA_TLB_IH_NONLEAF (((uint64_t)1) << 6)
#define DMA_TLB_MAX_SIZE (0x3f)

/* INVALID_DESC */
#define DMA_CCMD_INVL_GRANU_OFFSET  61
#define DMA_ID_TLB_GLOBAL_FLUSH	(((uint64_t)1) << 3)
#define DMA_ID_TLB_DSI_FLUSH	(((uint64_t)2) << 3)
#define DMA_ID_TLB_PSI_FLUSH	(((uint64_t)3) << 3)
#define DMA_ID_TLB_READ_DRAIN	(((uint64_t)1) << 7)
#define DMA_ID_TLB_WRITE_DRAIN	(((uint64_t)1) << 6)
#define DMA_ID_TLB_DID(id)	(((uint64_t)((id & 0xffff) << 16)))
#define DMA_ID_TLB_IH_NONLEAF	(((uint64_t)1) << 6)
#define DMA_ID_TLB_ADDR(addr)	(addr)
#define DMA_ID_TLB_ADDR_MASK(mask)	(mask)

/* PMEN_REG */
#define DMA_PMEN_EPM (((uint32_t)1)<<31)
#define DMA_PMEN_PRS (((uint32_t)1)<<0)

/* GCMD_REG */
#define DMA_GCMD_TE (((uint32_t)1) << 31)
#define DMA_GCMD_SRTP (((uint32_t)1) << 30)
#define DMA_GCMD_SFL (((uint32_t)1) << 29)
#define DMA_GCMD_EAFL (((uint32_t)1) << 28)
#define DMA_GCMD_WBF (((uint32_t)1) << 27)
#define DMA_GCMD_QIE (((uint32_t)1) << 26)
#define DMA_GCMD_SIRTP (((uint32_t)1) << 24)
#define DMA_GCMD_IRE (((uint32_t) 1) << 25)
#define DMA_GCMD_CFI (((uint32_t) 1) << 23)

/* GSTS_REG */
#define DMA_GSTS_TES (((uint32_t)1) << 31)
#define DMA_GSTS_RTPS (((uint32_t)1) << 30)
#define DMA_GSTS_FLS (((uint32_t)1) << 29)
#define DMA_GSTS_AFLS (((uint32_t)1) << 28)
#define DMA_GSTS_WBFS (((uint32_t)1) << 27)
#define DMA_GSTS_QIES (((uint32_t)1) << 26)
#define DMA_GSTS_IRTPS (((uint32_t)1) << 24)
#define DMA_GSTS_IRES (((uint32_t)1) << 25)
#define DMA_GSTS_CFIS (((uint32_t)1) << 23)

/* CCMD_REG */
#define DMA_CCMD_ICC (((uint64_t)1) << 63)
#define DMA_CCMD_GLOBAL_INVL (((uint64_t)1) << 61)
#define DMA_CCMD_DOMAIN_INVL (((uint64_t)2) << 61)
#define DMA_CCMD_DEVICE_INVL (((uint64_t)3) << 61)
#define DMA_CCMD_FM(m) (((uint64_t)((m) & 0x3)) << 32)
#define DMA_CCMD_MASK_NOBIT 0
#define DMA_CCMD_MASK_1BIT 1
#define DMA_CCMD_MASK_2BIT 2
#define DMA_CCMD_MASK_3BIT 3
#define DMA_CCMD_SID(s) (((uint64_t)((s) & 0xffff)) << 16)
#define DMA_CCMD_DID(d) ((uint64_t)((d) & 0xffff))

/* FECTL_REG */
#define DMA_FECTL_IM (((uint32_t)1) << 31)

/* FSTS_REG */
#define DMA_FSTS_PPF ((uint32_t)2)
#define DMA_FSTS_PFO ((uint32_t)1)
#define DMA_FSTS_IQE (1 << 4)
#define DMA_FSTS_ICE (1 << 5)
#define DMA_FSTS_ITE (1 << 6)
#define dma_fsts_fault_record_index(s) (((s) >> 8) & 0xff)

#endif
