/*
 * pcie_sriov.c:
 *
 * Implementation of SR/IOV emulation support.
 *
 * Copyright (c) 2014 Knut Omang <knut.omang@oracle.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pci_bus.h"
#include "qemu/range.h"

//#define DEBUG_SRIOV
#ifdef DEBUG_SRIOV
# define SRIOV_DPRINTF(fmt, ...)                                         \
    fprintf(stderr, "%s:%d " fmt, __func__, __LINE__, ## __VA_ARGS__)
#else
# define SRIOV_DPRINTF(fmt, ...) do {} while (0)
#endif
#define SRIOV_DEV_PRINTF(dev, fmt, ...)                                  \
    SRIOV_DPRINTF("%s:%x "fmt, (dev)->name, (dev)->devfn, ## __VA_ARGS__)

static PCIDevice *register_vf(PCIDevice *pf, int devfn, const char *name);
static void unregister_vfs(PCIDevice *dev);

void pcie_sriov_pf_init(PCIDevice *dev, uint16_t offset,
                        const char *vfname, uint16_t vf_dev_id,
                        uint16_t init_vfs, uint16_t total_vfs,
                        uint16_t vf_offset, uint16_t vf_stride)
{
    uint8_t *cfg = dev->config + offset;
    uint8_t *wmask;

    pcie_add_capability(dev, PCI_EXT_CAP_ID_SRIOV, 1,
                        offset, PCI_EXT_CAP_SRIOV_SIZEOF);
    dev->exp.sriov_cap = offset;
    dev->exp.sriov_pf.num_vfs = 0;
    dev->exp.sriov_pf.vfname = g_strdup(vfname);
    dev->exp.sriov_pf.vf = NULL;

    pci_set_word(cfg + PCI_SRIOV_VF_OFFSET, vf_offset);
    pci_set_word(cfg + PCI_SRIOV_VF_STRIDE, vf_stride);

    /* Minimal page size support values required by the SR/IOV standard:
     * 0x553 << 12 = 0x553000 = 4K + 8K + 64K + 256K + 1M + 4M
     * Hard coded for simplicity in this version
     */
    pci_set_word(cfg + PCI_SRIOV_SUP_PGSIZE, 0x553);

    /* Default is to use 4K pages, software can modify it
     * to any of the supported bits
     */
    pci_set_word(cfg + PCI_SRIOV_SYS_PGSIZE, 0x1);

    /* Set up device ID and initial/total number of VFs available */
    pci_set_word(cfg + PCI_SRIOV_VF_DID, vf_dev_id);
    pci_set_word(cfg + PCI_SRIOV_INITIAL_VF, init_vfs);
    pci_set_word(cfg + PCI_SRIOV_TOTAL_VF, total_vfs);
    pci_set_word(cfg + PCI_SRIOV_NUM_VF, 0);

    /* Write enable control bits */
    wmask = dev->wmask + offset;
    pci_set_word(wmask + PCI_SRIOV_CTRL,
                 PCI_SRIOV_CTRL_VFE | PCI_SRIOV_CTRL_MSE | PCI_SRIOV_CTRL_ARI);
    pci_set_word(wmask + PCI_SRIOV_NUM_VF, 0xffff);
    pci_set_word(wmask + PCI_SRIOV_SYS_PGSIZE, 0x553);

    qdev_prop_set_bit(&dev->qdev, "multifunction", true);
}

void pcie_sriov_pf_exit(PCIDevice *dev)
{
    SRIOV_DPRINTF("\n");
    unregister_vfs(dev);
    g_free((char *)dev->exp.sriov_pf.vfname);
    dev->exp.sriov_pf.vfname = NULL;
}

void pcie_sriov_pf_init_vf_bar(PCIDevice *dev, int region_num,
                               uint8_t type, dma_addr_t size)
{
    uint32_t addr;
    uint64_t wmask;
    uint16_t sriov_cap = dev->exp.sriov_cap;

    assert(sriov_cap > 0);
    assert(region_num >= 0);
    assert(region_num < PCI_NUM_REGIONS);
    assert(region_num != PCI_ROM_SLOT);

    wmask = ~(size - 1);
    addr = sriov_cap + PCI_SRIOV_BAR + region_num * 4;

    pci_set_long(dev->config + addr, type);
    if (!(type & PCI_BASE_ADDRESS_SPACE_IO) &&
        type & PCI_BASE_ADDRESS_MEM_TYPE_64) {
        pci_set_quad(dev->wmask + addr, wmask);
        pci_set_quad(dev->cmask + addr, ~0ULL);
    } else {
        pci_set_long(dev->wmask + addr, wmask & 0xffffffff);
        pci_set_long(dev->cmask + addr, 0xffffffff);
    }
    dev->exp.sriov_pf.vf_bar_type[region_num] = type;
}

void pcie_sriov_vf_register_bar(PCIDevice *dev, int region_num,
                                MemoryRegion *memory)
{
    PCIIORegion *r;
    uint8_t type;
    pcibus_t size = memory_region_size(memory);

    assert(pci_is_vf(dev)); /* PFs must use pci_register_bar */
    assert(region_num >= 0);
    assert(region_num < PCI_NUM_REGIONS);
    type = dev->exp.sriov_vf.pf->exp.sriov_pf.vf_bar_type[region_num];

    if (size & (size-1)) {
        fprintf(stderr, "ERROR: PCI region size must be a power"
                " of two - type=0x%x, size=0x%"FMT_PCIBUS"\n", type, size);
        exit(1);
    }

    r = &dev->io_regions[region_num];
    r->memory = memory;
    r->address_space =
        type & PCI_BASE_ADDRESS_SPACE_IO
        ? dev->bus->address_space_io
        : dev->bus->address_space_mem;
    r->size = size;
    r->type = type;

    r->addr = pci_bar_address(dev, region_num, r->type, r->size);
    if (r->addr != PCI_BAR_UNMAPPED) {
        memory_region_add_subregion_overlap(r->address_space,
                                            r->addr, r->memory, 1);
    }
}

static PCIDevice *register_vf(PCIDevice *pf, int devfn, const char *name)
{
    PCIDevice *dev = pci_create(pf->bus, devfn, name);
    dev->exp.sriov_vf.pf = pf;
    Error *local_err = NULL;

    object_property_set_bool(OBJECT(&dev->qdev), true, "realized", &local_err);
    if (local_err) {
        fprintf(stderr, "Failed to enable: %s\n",
                error_get_pretty(local_err));
        error_free(local_err);
        return NULL;
    }

    /* set vid/did according to sr/iov spec - they are not used */
    pci_config_set_vendor_id(dev->config, 0xffff);
    pci_config_set_device_id(dev->config, 0xffff);
    return dev;
}

static void register_vfs(PCIDevice *dev)
{
    uint16_t num_vfs;
    uint16_t i;
    uint16_t sriov_cap = dev->exp.sriov_cap;
    uint16_t vf_offset = pci_get_word(dev->config + sriov_cap + PCI_SRIOV_VF_OFFSET);
    uint16_t vf_stride = pci_get_word(dev->config + sriov_cap + PCI_SRIOV_VF_STRIDE);
    int32_t devfn = dev->devfn + vf_offset;

    assert(sriov_cap > 0);
    num_vfs = pci_get_word(dev->config + sriov_cap + PCI_SRIOV_NUM_VF);

    dev->exp.sriov_pf.vf = g_malloc(sizeof(PCIDevice *) * num_vfs);
    assert(dev->exp.sriov_pf.vf);

    SRIOV_DEV_PRINTF(dev, "creating %d vf devs\n", num_vfs);
    for (i = 0; i < num_vfs; i++) {
        dev->exp.sriov_pf.vf[i] = register_vf(dev, devfn, dev->exp.sriov_pf.vfname);
        if (!dev->exp.sriov_pf.vf[i]) {
            SRIOV_DEV_PRINTF(dev, "Failed to create VF %d at devfn %x.%x\n", i,
                             PCI_SLOT(devfn), PCI_FUNC(devfn));
            num_vfs = i;
            break;
        }
        devfn += vf_stride;
    }
    dev->exp.sriov_pf.num_vfs = num_vfs;
}

static void unregister_vfs(PCIDevice *dev)
{
    Error *local_err = NULL;
    uint16_t num_vfs = dev->exp.sriov_pf.num_vfs;
    uint16_t i;
    SRIOV_DEV_PRINTF(dev, "Unregistering %d vf devs\n", num_vfs);
    for (i = 0; i < num_vfs; i++) {
        object_property_set_bool(OBJECT(&dev->exp.sriov_pf.vf[i]->qdev), false, "realized", &local_err);
        if (local_err) {
            fprintf(stderr, "Failed to unplug: %s\n",
                    error_get_pretty(local_err));
            error_free(local_err);
        }
    }
    g_free(dev->exp.sriov_pf.vf);
    dev->exp.sriov_pf.vf = NULL;
    dev->exp.sriov_pf.num_vfs = 0;
}

void pcie_sriov_config_write(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    uint32_t off;
    uint16_t sriov_cap = dev->exp.sriov_cap;

    if (!sriov_cap || address < sriov_cap) {
        return;
    }
    off = address - sriov_cap;
    if (off >= PCI_EXT_CAP_SRIOV_SIZEOF) {
        return;
    }

    SRIOV_DEV_PRINTF(dev, "cap at 0x%x sriov offset 0x%x val 0x%x len %d\n",
                 sriov_cap, off, val, len);

    if (range_covers_byte(off, len, PCI_SRIOV_CTRL)) {
        if (dev->exp.sriov_pf.num_vfs) {
            if (!(val & PCI_SRIOV_CTRL_VFE)) {
                unregister_vfs(dev);
            }
        } else {
            if (val & PCI_SRIOV_CTRL_VFE) {
                register_vfs(dev);
            }
        }
    }
}


/* Reset SR/IOV VF Enable bit to trigger an unregister of all VFs */
void pcie_sriov_pf_disable_vfs(PCIDevice *dev)
{
    uint16_t sriov_cap = dev->exp.sriov_cap;
    if (sriov_cap) {
        uint32_t val = pci_get_byte(dev->config + sriov_cap + PCI_SRIOV_CTRL);
        if (val & PCI_SRIOV_CTRL_VFE) {
            val &= ~PCI_SRIOV_CTRL_VFE;
            pcie_sriov_config_write(dev, sriov_cap + PCI_SRIOV_CTRL, val, 1);
        }
    }
}

/* Add optional supported page sizes to the mask of supported page sizes */
void pcie_sriov_pf_add_sup_pgsize(PCIDevice *dev, uint16_t opt_sup_pgsize)
{
    uint8_t *cfg = dev->config + dev->exp.sriov_cap;
    uint8_t *wmask = dev->wmask + dev->exp.sriov_cap;

    uint16_t sup_pgsize = pci_get_word(cfg + PCI_SRIOV_SUP_PGSIZE);

    sup_pgsize |= opt_sup_pgsize;

    /* Make sure the new bits are set, and that system page size
     * also can be set to any of the new values according to spec:
     */
    pci_set_word(cfg + PCI_SRIOV_SUP_PGSIZE, sup_pgsize);
    pci_set_word(wmask + PCI_SRIOV_SYS_PGSIZE, sup_pgsize);
}
