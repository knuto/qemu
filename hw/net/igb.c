/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Intel developer's manual for these devices:
 * http://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82576eg-gbe-datasheet.pdf
 *
 * Copied and edited from the E1000e QEMU emulation (e1000e.c) by Knut Omang.
 *
 * The E1000e code is authored by:
 * Dmitry Fleytman <dmitry@daynix.com>
 * Leonid Bloch <leonid@daynix.com>
 * Yan Vugenfirer <yan@daynix.com>
 *
 * Copyright (c) 2015 Ravello Systems LTD (http://ravellosystems.com)
 * Developed by Daynix Computing LTD (http://www.daynix.com)
 * Nir Peleg, Tutis Systems Ltd. for Qumranet Inc.
 * Copyright (c) 2008 Qumranet
 * Based on work done by:
 * Copyright (c) 2007 Dan Aloni
 * Copyright (c) 2004 Antony T Curtis
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

#include "qemu/osdep.h"
#include "qemu/range.h"
#include "sysemu/sysemu.h"
#include "net/net.h"
#include "linux/virtio_net.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/net/igb_regs.h"
#include "e1000x_common.h"
#include "e1000e_core.h"

#include "trace.h"
#include "qapi/error.h"

#define TYPE_IGB "igb"
#define IGB(obj)   OBJECT_CHECK(IgbState, (obj), TYPE_IGB)
#define TYPE_IGBVF "igbvf"
#define IGBVF(obj) OBJECT_CHECK(IgbState, (obj), TYPE_IGBVF)

#define IGB_MSIX_BAR 3
#define IGB_MSIX_VECTORS_PF 10
#define IGB_MSIX_VECTORS_VF 3
#define IGB_CAP_SRIOV_OFFSET 0x160
#define IGB_TOTAL_VFS 8
#define IGB_VF_OFFSET 0x80
#define IGB_VF_STRIDE 2

typedef struct IgbState {
    PCIDevice parent_obj;
    NICState *nic;
    NICConf conf;

    MemoryRegion mmio;
    MemoryRegion flash;
    MemoryRegion io;
    MemoryRegion msix;

    uint32_t ioaddr;

    uint16_t subsys_ven;
    uint16_t subsys;

    uint16_t subsys_ven_used;
    uint16_t subsys_used;

    E1000ECore core;
} IgbState;

#define IGB_MMIO_SIZE    (128 * 1024)
#define IGB_FLASH_SIZE   (128 * 1024)
#define IGB_IO_SIZE      (32)

static void igb_write_config(PCIDevice *d, uint32_t address,
                             uint32_t val, int len)
{
    IgbState *s = IGB(d);
    trace_igb_write_config(address, val, len);
    pci_default_write_config(d, address, val, len);

    if (range_covers_byte(address, len, PCI_COMMAND) &&
        (d->config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        e1000e_start_recv(&s->core);
    }
}

static void igbvf_write_config(PCIDevice *d, uint32_t address,
                             uint32_t val, int len)
{
    IgbState *s = IGBVF(d);
    (void)s;
    trace_igbvf_write_config(address, val, len);
    pci_default_write_config(d, address, val, len);

    if (range_covers_byte(address, len, PCI_COMMAND) &&
        (d->config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        //e1000e_start_recv(&s->core);
    }
}

static uint64_t igb_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbState *s = opaque;
    return e1000e_core_read(&s->core, addr, size);
}

static void igb_mmio_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    IgbState *s = opaque;
    e1000e_core_write(&s->core, addr, val, size);
}

static bool igb_io_get_reg_index(IgbState *s, uint32_t *idx)
{
    if (s->ioaddr < 0x1FFFF) {
        *idx = s->ioaddr;
        return true;
    }

    if (s->ioaddr < 0x7FFFF) {
        trace_igb_wrn_io_addr_undefined(s->ioaddr);
        return false;
    }

    if (s->ioaddr < 0xFFFFF) {
        trace_igb_wrn_io_addr_flash(s->ioaddr);
        return false;
    }

    trace_igb_wrn_io_addr_unknown(s->ioaddr);
    return false;
}

static uint64_t igb_io_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbState *s = opaque;
    uint32_t idx = 0;
    uint64_t val;

    switch (addr) {
    case E1000_IOADDR:
        trace_igb_io_read_addr(s->ioaddr);
        return s->ioaddr;
    case E1000_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            val = e1000e_core_read(&s->core, idx, sizeof(val));
            trace_igb_io_read_data(idx, val);
            return val;
        }
        return 0;
    default:
        trace_igb_wrn_io_read_unknown(addr);
        return 0;
    }
}

static void igb_io_write(void *opaque, hwaddr addr,
                         uint64_t val, unsigned size)
{
    IgbState *s = opaque;
    uint32_t idx = 0;

    switch (addr) {
    case E1000_IOADDR:
        trace_igb_io_write_addr(val);
        s->ioaddr = (uint32_t) val;
        return;
    case E1000_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            trace_igb_io_write_data(idx, val);
            e1000e_core_write(&s->core, idx, val, sizeof(val));
        }
        return;
    default:
        trace_igb_wrn_io_write_unknown(addr);
        return;
    }
}

static const MemoryRegionOps mmio_ops = {
    .read = igb_mmio_read,
    .write = igb_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps io_ops = {
    .read = igb_io_read,
    .write = igb_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool igb_nc_can_receive(NetClientState *nc)
{
    // IgbState *s = qemu_get_nic_opaque(nc);
    return 0; //e1000e_can_receive(&s->core);
}

static ssize_t igb_nc_receive_iov(NetClientState *nc, const struct iovec *iov, int iovcnt)
{
    // IgbState *s = qemu_get_nic_opaque(nc);
    return 0; //igb_receive_iov(&s->core, iov, iovcnt);
}

static ssize_t igb_nc_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    // IgbState *s = qemu_get_nic_opaque(nc);
    return 0; //igb_receive(&s->core, buf, size);
}

static void igb_set_link_status(NetClientState *nc)
{
    // IgbState *s = qemu_get_nic_opaque(nc);
    // igb_core_set_link_status(&s->core);
}


static NetClientInfo net_igb_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = igb_nc_can_receive,
    .receive = igb_nc_receive,
    .receive_iov = igb_nc_receive_iov,
    .link_status_changed = igb_set_link_status,
};


static int igb_add_pm_capability(PCIDevice *pdev, uint8_t offset, uint16_t pmc)
{
    Error *local_err = NULL;
    int ret = pci_add_capability(pdev, PCI_CAP_ID_PM, offset,
                                 PCI_PM_SIZEOF, &local_err);

    if (local_err) {
        error_report_err(local_err);
        return ret;
    }

    pci_set_word(pdev->config + offset + PCI_PM_PMC,
                 PCI_PM_CAP_VER_1_1 |
                 pmc);

    pci_set_word(pdev->wmask + offset + PCI_PM_CTRL,
                 PCI_PM_CTRL_STATE_MASK |
                 PCI_PM_CTRL_PME_ENABLE |
                 PCI_PM_CTRL_DATA_SEL_MASK);

    pci_set_word(pdev->w1cmask + offset + PCI_PM_CTRL,
                 PCI_PM_CTRL_PME_STATUS);

    return ret;
}

static void igb_init_net_peer(IgbState *s, PCIDevice *pci_dev, uint8_t *macaddr)
{
    DeviceState *dev = DEVICE(pci_dev);
    NetClientState *nc;
    int i;

    s->nic = qemu_new_nic(&net_igb_info, &s->conf,
        object_get_typename(OBJECT(s)), dev->id, s);

    s->core.max_queue_num = s->conf.peers.queues - 1;

    trace_igb_mac_set_permanent(MAC_ARG(macaddr));
    memcpy(s->core.permanent_mac, macaddr, sizeof(s->core.permanent_mac));

    qemu_format_nic_info_str(qemu_get_queue(s->nic), macaddr);
    s->core.has_vnet = true;

    for (i = 0; i < s->conf.peers.queues; i++) {
        nc = qemu_get_subqueue(s->nic, i);
        if (!nc->peer || !qemu_has_vnet_hdr(nc->peer)) {
            s->core.has_vnet = false;
            trace_igb_cfg_support_virtio(false);
            return;
        }
    }

    trace_igb_cfg_support_virtio(true);

    for (i = 0; i < s->conf.peers.queues; i++) {
        nc = qemu_get_subqueue(s->nic, i);
        qemu_set_vnet_hdr_len(nc->peer, sizeof(struct virtio_net_hdr));
        qemu_using_vnet_hdr(nc->peer, true);
    }
}

/* EEPROM (NVM) contents documented in section 6.1, table 6-1:
 * and in 6.10 Software accessed words.
 *
 * TBD: Need to walk through this, names in comments are ok up to 0x4F
 */
static const uint16_t igb_eeprom_template[80] = {
  /*        Address        |    Compat.    | ImRev |Compat.|OEM sp.*/
    0x0000, 0x0000, 0x0000, 0x0d14, 0xffff, 0x2010, 0xffff, 0xffff,
  /*      PBA      |ICtrl1 | SSID  | SVID  | DevID |-------|ICtrl2 */
    0x1040, 0xffff, 0x046b, 0x484c, 0x108e, 0x10c9, 0x0000, 0xf14b,
  /* SwPin0| DevID | EESZ  |-------|ICtrl3 |PCI-tc | MSIX  | APtr  */
    0xe30c, 0x10c9, 0x6000, 0x0000, 0x8c01, 0x0014, 0x4a40, 0x0060,
  /* PCIe Init. Conf 1,2,3 |PCICtrl| LD1,3 |DDevID |DevRev | LD0,2 */
    0x6cf6, 0xd7b0, 0x0a7e, 0x8403, 0x4784, 0x10a6, 0x0001, 0x4602,
  /* SwPin1| FunC  |LAN-PWR|ManHwC |ICtrl3 | IOVct |VDevID |-------*/
    0xe30c, 0x2020, 0x1ae5, 0x004a, 0x8401, 0x00f7, 0x10ca, 0x0000,
  /*---------------| LD1,3 | LD0,2 | ROEnd | ROSta | Wdog  | VPD   */
    0x0000, 0x0000, 0x4784, 0x4602, 0x0000, 0x0000, 0x0000, 0xffff,
  /* PCSet0| Ccfg0 |PXEver |IBAcap |PCSet1 | Ccfg1 |iSCVer | ??    */
    0x0100, 0x4000, 0x131f, 0x4013, 0x0100, 0x4000, 0xffff, 0xffff,
  /* PCSet2| Ccfg2 |PCSet3 | Ccfg3 | ??    |AltMacP| ??    |CHKSUM */
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x00e0, 0xffff, 0xb73b,
  /* ArbEn |-------| ImuID | ImuID |-------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  /*----------------------- Reserved ------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  /* Word 0x50 - 0x5XX (sec.6.5) */
};


static void pci_igb_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbState *igb = IGB(d);
    uint8_t *macaddr;

    trace_igb_cb_pci_realize();

    d->config_write = igb_write_config;

    d->config[PCI_CACHE_LINE_SIZE] = 0x10;
    d->config[PCI_INTERRUPT_PIN] = 1;

    /* BAR0: MMIO */
    memory_region_init_io(&igb->mmio, OBJECT(d), &mmio_ops, igb,
                          "igb-mmio", IGB_MMIO_SIZE);
    pci_register_bar(d, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &igb->mmio);

    /* BAR1: flash memory (dummy) */
    memory_region_init(&igb->flash, OBJECT(d),
                          "igb-flash", IGB_FLASH_SIZE);
    pci_register_bar(d, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &igb->flash);

    /* BAR2: I/O ports */
    memory_region_init_io(&igb->io, OBJECT(d), &io_ops, igb,
                          "igb-io", IGB_IO_SIZE);
    pci_register_bar(d, 2, PCI_BASE_ADDRESS_SPACE_IO, &igb->io);

    /* BAR3: MSIX table */
    memory_region_init(&igb->msix, OBJECT(d), "igb-msix", 0x4000);
    pci_register_bar(d, 3, PCI_BASE_ADDRESS_MEM_TYPE_64, &igb->msix);

    /* Add PCI capabilities in reverse order: */
    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = msix_init(d, IGB_MSIX_VECTORS_PF, &igb->msix, IGB_MSIX_BAR, 0, &igb->msix,
                    IGB_MSIX_BAR, 0x2000, 0x70, errp);
    if (ret) {
        goto err_msix;
    }

    ret = msi_init(d, 0x50, 1, true, true, errp);
    if (ret < 0) {
        goto err_msi;
    }

    /* TBD: Only initialize the vectors used */
    for (v = 0; v < IGB_MSIX_VECTORS_PF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    if (igb_add_pm_capability(d, 0x40, PCI_PM_CAP_DSI) < 0) {
        hw_error("Failed to initialize PM capability");
    }

    /* PCIe extended capabilities (in order) */
    ret = pcie_aer_init(d, 1, 0x100, 0x40, errp);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, 0x150, 1);

    pcie_sriov_pf_init(d, IGB_CAP_SRIOV_OFFSET, "igbvf",
                       IGB_82576_VF_DEV_ID, IGB_TOTAL_VFS, IGB_TOTAL_VFS,
                       IGB_VF_OFFSET, IGB_VF_STRIDE);

    pcie_sriov_pf_init_vf_bar(d, 0, PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                              0x8000);
    pcie_sriov_pf_init_vf_bar(d, 3, PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                              0x8000);

    /* TBD: simple network stack side setup - rudely copied from e1000e.c
     */

    /* Create networking backend */
    qemu_macaddr_default_if_unset(&igb->conf.macaddr);
    macaddr = igb->conf.macaddr.a;

    igb_init_net_peer(igb, d, macaddr);

    /* Initialize core */
    igb->core.owner = &igb->parent_obj;
    igb->core.owner_nic = igb->nic;

    e1000e_core_pci_realize(&igb->core,
                            igb_eeprom_template,
                            sizeof(igb_eeprom_template),
                            macaddr);
    return;
 err_aer:
    msi_uninit(d);
 err_msi:
    msix_unuse_all_vectors(d);
    msix_uninit(d, &igb->msix, &igb->msix);
 err_msix:
    pcie_cap_exit(d);
 err_pcie_cap:
    return;
    /* TBD: pci_e1000_uninit(d); */
}

static void pci_igb_uninit(PCIDevice *d)
{
    IgbState *igb = IGB(d);
    MemoryRegion *mr = &igb->msix;

    trace_igb_cb_pci_uninit();

    pcie_sriov_pf_exit(d);
    pcie_cap_exit(d);
    qemu_del_nic(igb->nic);
    msix_unuse_all_vectors(d);
    msix_uninit(d, mr, mr);
    msi_uninit(d);
}

static void igb_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    IgbState *s = IGB(dev);

    trace_igb_cb_qdev_reset();
    pcie_sriov_pf_disable_vfs(d);
    e1000e_core_reset(&s->core);

    /* On the igb, the SMBI bit is 0 at reset, while on the e1000e
     * it is set to one, correct this:
     */
    s->core.mac[SWSM] = 0;

    s->core.mac[EEMNGCTL] |= E1000_EEPROM_CFG_DONE | E1000_EEPROM_CFG_DONE_PORT_1;
    s->core.phy[0][PHY_ID1] = 0x2a8;
    s->core.phy[0][PHY_ID2] = 0x391;
}

static int igb_pre_save(void *opaque)
{
    IgbState *s = opaque;

    trace_igb_cb_pre_save();

    e1000e_core_pre_save(&s->core);
    return 0;
}

static int igb_post_load(void *opaque, int version_id)
{
    IgbState *s = opaque;

    trace_igb_cb_post_load();

    if ((s->subsys != s->subsys_used) ||
        (s->subsys_ven != s->subsys_ven_used)) {
        fprintf(stderr,
            "ERROR: Cannot migrate while device properties "
            "(subsys/subsys_ven) differ");
        return -1;
    }
    return e1000e_core_post_load(&s->core);
}

static void pci_igbvf_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbState *igb = IGBVF(d);
    MemoryRegion *mr = &igb->msix;
    /* TBD: pci_e1000_realize(d, errp); */
    if (*errp)
        return;

    d->config_write = igbvf_write_config;

    memory_region_init(mr, OBJECT(d), "igbvf-msix", 0x8000);
    pcie_sriov_vf_register_bar(d, 3, mr);
    ret = msix_init(d, IGB_MSIX_VECTORS_VF, mr, IGB_MSIX_BAR, 0, mr,
                    IGB_MSIX_BAR, 0x2000, 0x70, errp);
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

    ret = pcie_aer_init(d, 1, 0x100, 0x40, errp);
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
    return;
    /* TBD: pci_e1000_uninit(d); */
}

static void igbvf_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    IgbState *s = IGBVF(dev);

    (void)s;
    (void)d;
    trace_igb_cb_qdev_reset();

    /* TBD */
}


static void pci_igbvf_uninit(PCIDevice *d)
{
    IgbState *igb = IGBVF(d);
    MemoryRegion *mr = &igb->msix;

    pcie_cap_exit(d);
    msix_uninit(d, mr, mr);
    /* TBD: pci_e1000_uninit(d); */
}


static const VMStateDescription igb_vmstate_tx = {
    .name = "igb-tx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(sum_needed, struct e1000e_tx),
        VMSTATE_UINT8(props.ipcss, struct e1000e_tx),
        VMSTATE_UINT8(props.ipcso, struct e1000e_tx),
        VMSTATE_UINT16(props.ipcse, struct e1000e_tx),
        VMSTATE_UINT8(props.tucss, struct e1000e_tx),
        VMSTATE_UINT8(props.tucso, struct e1000e_tx),
        VMSTATE_UINT16(props.tucse, struct e1000e_tx),
        VMSTATE_UINT8(props.hdr_len, struct e1000e_tx),
        VMSTATE_UINT16(props.mss, struct e1000e_tx),
        VMSTATE_UINT32(props.paylen, struct e1000e_tx),
        VMSTATE_INT8(props.ip, struct e1000e_tx),
        VMSTATE_INT8(props.tcp, struct e1000e_tx),
        VMSTATE_BOOL(props.tse, struct e1000e_tx),
        VMSTATE_BOOL(cptse, struct e1000e_tx),
        VMSTATE_BOOL(skip_cp, struct e1000e_tx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription igb_vmstate_intr_timer = {
    .name = "e1000e-intr-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, E1000IntrDelayTimer),
        VMSTATE_BOOL(running, E1000IntrDelayTimer),
        VMSTATE_END_OF_LIST()
    }
};

#define VMSTATE_E1000E_INTR_DELAY_TIMER(_f, _s)                     \
    VMSTATE_STRUCT(_f, _s, 0,                                       \
                   igb_vmstate_intr_timer, E1000IntrDelayTimer)

#define VMSTATE_E1000E_INTR_DELAY_TIMER_ARRAY(_f, _s, _num)         \
    VMSTATE_STRUCT_ARRAY(_f, _s, _num, 0,                           \
                         igb_vmstate_intr_timer, E1000IntrDelayTimer)

static const VMStateDescription igb_vmstate = {
    .name = "e1000e",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = igb_pre_save,
    .post_load = igb_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IgbState),
        VMSTATE_MSIX(parent_obj, IgbState),

        VMSTATE_UINT32(ioaddr, IgbState),
        VMSTATE_UINT32(core.rxbuf_min_shift, IgbState),
        VMSTATE_UINT8(core.rx_desc_len, IgbState),
        VMSTATE_UINT32_ARRAY(core.rxbuf_sizes, IgbState,
                             E1000_PSRCTL_BUFFS_PER_DESC),
        VMSTATE_UINT32(core.rx_desc_buf_size, IgbState),
        VMSTATE_UINT16_ARRAY(core.eeprom, IgbState, E1000E_EEPROM_SIZE),
        VMSTATE_UINT16_2DARRAY(core.phy, IgbState,
                               E1000E_PHY_PAGES, E1000E_PHY_PAGE_SIZE),
        VMSTATE_UINT32_ARRAY(core.mac, IgbState, E1000E_MAC_SIZE),
        VMSTATE_UINT8_ARRAY(core.permanent_mac, IgbState, ETH_ALEN),

        VMSTATE_UINT32(core.delayed_causes, IgbState),

        VMSTATE_UINT16(subsys, IgbState),
        VMSTATE_UINT16(subsys_ven, IgbState),
        VMSTATE_E1000E_INTR_DELAY_TIMER(core.rdtr, IgbState),
        VMSTATE_E1000E_INTR_DELAY_TIMER(core.radv, IgbState),
        VMSTATE_E1000E_INTR_DELAY_TIMER(core.raid, IgbState),
        VMSTATE_E1000E_INTR_DELAY_TIMER(core.tadv, IgbState),
        VMSTATE_E1000E_INTR_DELAY_TIMER(core.tidv, IgbState),

        VMSTATE_E1000E_INTR_DELAY_TIMER(core.itr, IgbState),
        VMSTATE_BOOL(core.itr_intr_pending, IgbState),

        VMSTATE_E1000E_INTR_DELAY_TIMER_ARRAY(core.eitr, IgbState,
                                              E1000E_MSIX_VEC_NUM),
        VMSTATE_BOOL_ARRAY(core.eitr_intr_pending, IgbState,
                           E1000E_MSIX_VEC_NUM),

        VMSTATE_UINT32(core.itr_guest_value, IgbState),
        VMSTATE_UINT32_ARRAY(core.eitr_guest_value, IgbState,
                             E1000E_MSIX_VEC_NUM),

        VMSTATE_UINT16(core.vet, IgbState),

        VMSTATE_STRUCT_ARRAY(core.tx, IgbState, E1000E_NUM_QUEUES, 0,
                             igb_vmstate_tx, struct e1000e_tx),
        VMSTATE_END_OF_LIST()
    }
};

static PropertyInfo igb_prop_subsys_ven,
                    igb_prop_subsys;

static Property igb_properties[] = {
    DEFINE_NIC_PROPERTIES(IgbState, conf),
    DEFINE_PROP_SIGNED("subsys_ven", IgbState, subsys_ven,
                        PCI_VENDOR_ID_INTEL,
                        igb_prop_subsys_ven, uint16_t),
    DEFINE_PROP_SIGNED("subsys", IgbState, subsys, 0,
                        igb_prop_subsys, uint16_t),
    DEFINE_PROP_END_OF_LIST(),
};

static void igb_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = pci_igb_realize;
    c->exit = pci_igb_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = E1000_DEV_ID_82576;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 GbE Controller";
    dc->reset = igb_reset;
    dc->vmsd = &igb_vmstate;

    igb_prop_subsys_ven = qdev_prop_uint16;
    igb_prop_subsys_ven.description = "PCI device Subsystem Vendor ID";

    igb_prop_subsys = qdev_prop_uint16;
    igb_prop_subsys.description = "PCI device Subsystem ID";

    device_class_set_props(dc, igb_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}


static void igb_instance_init(Object * obj)
{
    IgbState *s = IGB(obj);
    device_add_bootindex_property(obj, &s->conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj));
}

static void igbvf_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = pci_igbvf_realize;
    c->exit = pci_igbvf_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = IGB_82576_VF_DEV_ID;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 GbE Controller Virtual Function";
    dc->reset = igbvf_reset;
    dc->vmsd = &igb_vmstate;
    device_class_set_props(dc, igb_properties);
}


static void igbvf_instance_init(Object * obj)
{

}

static const TypeInfo igb_info = {
    .name = TYPE_IGB,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbState),
    .class_init = igb_class_init,
    .instance_init = igb_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static const TypeInfo igbvf_info = {
    .name = TYPE_IGBVF,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbState),
    .class_init = igbvf_class_init,
    .instance_init = igbvf_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};


static void igb_register_types(void)
{
    type_register_static(&igb_info);
    type_register_static(&igbvf_info);
}

type_init(igb_register_types)
