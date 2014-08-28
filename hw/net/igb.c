/*
 * igb.c
 *
 * Intel 82576 Gigabit Ethernet Adapter
 * (SR/IOV capable PCIe ethernet device) emulation
 *
 * Copyright (c) 2014 Knut Omang <knut.omang@oracle.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

/* NB! This implementation does not work (yet) as an ethernet device,
 * it mainly serves as an example to demonstrate the emulated SR/IOV device
 * building blocks!
 *
 * You should be able to load both the PF and VF driver (igb/igbvf) on Linux
 * if the igb driver is loaded with a num_vfs != 0 driver parameter.
 * but the PF fails to reset (probably because that where the obvious
 * similarities between e1000 and igb ends..
 */

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "net/net.h"
#include "igb_regs.h"
#include "e1000.h"

typedef struct IgbState {
    /*< private >*/
    E1000State parent_obj;

    /*< public >*/
    MemoryRegion flash;
    MemoryRegion msix;
} IgbState;




static uint64_t igb_flash_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    fprintf(stderr, "%s: addr %lx size %d (Not implemented)\n",
            __func__, addr, size);
    return 0;
}

static void igb_flash_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    fprintf(stderr, "%s: addr %lx size %d, value %lx (Not implemented)\n",
            __func__, addr, size, val);
}

static const MemoryRegionOps igb_flash_ops = {
    .read = igb_flash_read,
    .write = igb_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};


#define TYPE_IGB "igb"
#define IGB(obj) \
    OBJECT_CHECK(IgbState, (obj), TYPE_IGB)
#define TYPE_IGBVF "igbvf"
#define IGBVF(obj) \
    OBJECT_CHECK(IgbState, (obj), TYPE_IGBVF)

static void pci_igb_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbState *igb = IGB(d);
    MemoryRegion *mr = &igb->flash;
    Error *local_err = NULL;
    pci_e1000_realize(d, &local_err);
    if (*errp)
        return;

    memory_region_init_io(mr, OBJECT(d), &igb_flash_ops,
                          igb, "igb-flash", 0x20000);
    pci_register_bar(d, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &igb->flash);

    ret = msi_init(d, 0x40, 2, true, true);
    if (ret < 0) {
        goto err_msi;
    }

    mr = &igb->msix;
    memory_region_init(mr, OBJECT(d), "igb-msix", 0x8000);
    pci_register_bar(d, 3, PCI_BASE_ADDRESS_MEM_TYPE_64, mr);
    ret = msix_init(d, IGB_MSIX_VECTORS_PF, mr, IGB_MSIX_BAR, 0, mr,
                    IGB_MSIX_BAR, 0x2000, 0x70);
    if (ret) {
        goto err_msix;
    }

    /* TBD: Only initialize the vectors used */
    for (v = 0; v < IGB_MSIX_VECTORS_PF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }
    ret = pcie_aer_init(d, 0x100);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, 0x150, 1);

    pcie_sriov_pf_init(d, IGB_CAP_SRIOV_OFFSET, "igbvf",
                       IGB_82576_VF_DEV_ID, IGB_TOTAL_VFS, IGB_TOTAL_VFS,
                       IGB_VF_OFFSET, IGB_VF_STRIDE);

    pcie_sriov_pf_init_vf_bar(d, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, 0x8000);
    pcie_sriov_pf_init_vf_bar(d, 3, PCI_BASE_ADDRESS_MEM_TYPE_64, 0x8000);

    return;
 err_aer:
    pcie_cap_exit(d);
 err_pcie_cap:
    msix_unuse_all_vectors(d);
    msix_uninit(d, mr, mr);
 err_msix:
    msi_uninit(d);
 err_msi:
    pci_e1000_uninit(d);
}


static void pci_igb_uninit(PCIDevice *d)
{
    IgbState *igb = IGB(d);
    MemoryRegion *mr = &igb->msix;

    pcie_sriov_pf_exit(d);
    pcie_cap_exit(d);
    msix_unuse_all_vectors(d);
    msix_uninit(d, mr, mr);
    msi_uninit(d);
    pci_e1000_uninit(d);
}

static void igb_reset(DeviceState *dev)
{
    E1000State *ed = E1000(dev);
    PCIDevice *d = PCI_DEVICE(dev);
    pcie_sriov_pf_disable_vfs(d);
    e1000_reset(ed);
}


static void pci_igbvf_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbState *igb = IGBVF(d);
    MemoryRegion *mr = &igb->flash;
    pci_e1000_realize(d, errp);
    if (*errp)
        return;

    mr = &igb->msix;
    memory_region_init(mr, OBJECT(d), "igbvf-msix", 0x8000);
    pcie_sriov_vf_register_bar(d, 3, mr);
    ret = msix_init(d, IGB_MSIX_VECTORS_VF, mr, IGB_MSIX_BAR, 0, mr,
                    IGB_MSIX_BAR, 0x2000, 0x70);
    if (ret) {
        goto err_msix;
    }

    for (v = 0; v < IGB_MSIX_VECTORS_VF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = pcie_aer_init(d, 0x100);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, 0x150, 1);
    return;

 err_aer:
    pcie_cap_exit(d);
 err_pcie_cap:
    msix_unuse_all_vectors(d);
    msix_uninit(d, mr, mr);
 err_msix:
    pci_e1000_uninit(d);
}


static void pci_igbvf_uninit(PCIDevice *d)
{
    IgbState *igb = IGBVF(d);
    MemoryRegion *mr = &igb->msix;

    pcie_cap_exit(d);
    msix_uninit(d, mr, mr);
    pci_e1000_uninit(d);
}

static void igb_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    e1000_class_init(klass, data);
    /* extend/modify some methods/settings: */
    k->is_express = 1;
    k->realize = pci_igb_realize;
    k->exit = pci_igb_uninit;
    dc->reset = igb_reset;
}

static void igbvf_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    e1000_class_init(klass, data);

    /* extend/modify some methods/settings: */
    k->is_express = 1;
    k->romfile = NULL;
    k->realize = pci_igbvf_realize;
    k->exit = pci_igbvf_uninit;
}

static const E1000Info igb_device = {
    .name      = "igb",
    .device_id = E1000_DEV_ID_82576,
    .revision  = 0x01,
    .io_bar    = 2,
    .phy_id2   = I210_I_PHY_ID2,
};

static const E1000Info igbvf_device = {
    .name      = TYPE_IGBVF,
    .device_id = IGB_82576_VF_DEV_ID,
    .revision  = 0x01,
    .io_bar    = (uint8_t)-1,
    .phy_id2   = I210_I_PHY_ID2,
};


static const TypeInfo igb_info = {
    .name      = TYPE_IGB,
    .parent    = TYPE_E1000_BASE,
    .class_data = (void *) &igb_device,
    .class_init = igb_class_init,
    .instance_size = sizeof(IgbState),
};


static const TypeInfo igbvf_info = {
    .name      = "igbvf",
    .parent    = TYPE_E1000_BASE,
    .class_data = (void *) &igbvf_device,
    .class_init = igbvf_class_init,
    .instance_size = sizeof(IgbState),
};


static void igb_register_types(void)
{
    type_register_static(&igb_info);
    type_register_static(&igbvf_info);
}

type_init(igb_register_types)
