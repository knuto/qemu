/*
 * QEMU e1000 emulation
 *
 * Software developer's manual:
 * http://download.intel.com/design/network/manuals/8254x_GBe_SDM.pdf
 *
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

#include "e1000_regs.h"

typedef struct E1000State_st {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    NICState *nic;
    NICConf conf;
    MemoryRegion mmio;
    MemoryRegion io;

    uint32_t mac_reg[0x8000];
    uint16_t phy_reg[0x20];
    uint16_t eeprom_data[64];

    uint32_t rxbuf_size;
    uint32_t rxbuf_min_shift;
    struct e1000_tx {
        unsigned char header[256];
        unsigned char vlan_header[4];
        /* Fields vlan and data must not be reordered or separated. */
        unsigned char vlan[4];
        unsigned char data[0x10000];
        uint16_t size;
        unsigned char sum_needed;
        unsigned char vlan_needed;
        uint8_t ipcss;
        uint8_t ipcso;
        uint16_t ipcse;
        uint8_t tucss;
        uint8_t tucso;
        uint16_t tucse;
        uint8_t hdr_len;
        uint16_t mss;
        uint32_t paylen;
        uint16_t tso_frames;
        char tse;
        int8_t ip;
        int8_t tcp;
        char cptse;     // current packet tse bit
    } tx;

    struct {
        uint32_t val_in;	// shifted in from guest driver
        uint16_t bitnum_in;
        uint16_t bitnum_out;
        uint16_t reading;
        uint32_t old_eecd;
    } eecd_state;

    QEMUTimer *autoneg_timer;

    QEMUTimer *mit_timer;      /* Mitigation timer. */
    bool mit_timer_on;         /* Mitigation timer is running. */
    bool mit_irq_level;        /* Tracks interrupt pin level. */
    uint32_t mit_ide;          /* Tracks E1000_TXD_CMD_IDE bit. */

/* Compatibility flags for migration to/from qemu 1.3.0 and older */
#define E1000_FLAG_AUTONEG_BIT 0
#define E1000_FLAG_MIT_BIT 1
#define E1000_FLAG_AUTONEG (1 << E1000_FLAG_AUTONEG_BIT)
#define E1000_FLAG_MIT (1 << E1000_FLAG_MIT_BIT)
    uint32_t compat_flags;
} E1000State;

typedef struct E1000BaseClass {
    PCIDeviceClass parent_class;
    uint16_t phy_id2;
    uint16_t io_bar;  /* Which BAR register to use for the IO bar */
} E1000BaseClass;

#define TYPE_E1000_BASE "e1000-base"

#define E1000(obj) \
    OBJECT_CHECK(E1000State, (obj), TYPE_E1000_BASE)

#define E1000_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(E1000BaseClass, (klass), TYPE_E1000_BASE)
#define E1000_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(E1000BaseClass, (obj), TYPE_E1000_BASE)

#define	defreg(x)	x = (E1000_##x>>2)
enum {
    defreg(CTRL),	defreg(EECD),	defreg(EERD),	defreg(GPRC),
    defreg(GPTC),	defreg(ICR),	defreg(ICS),	defreg(IMC),
    defreg(IMS),	defreg(LEDCTL),	defreg(MANC),	defreg(MDIC),
    defreg(MPC),	defreg(PBA),	defreg(RCTL),	defreg(RDBAH),
    defreg(RDBAL),	defreg(RDH),	defreg(RDLEN),	defreg(RDT),
    defreg(STATUS),	defreg(SWSM),	defreg(TCTL),	defreg(TDBAH),
    defreg(TDBAL),	defreg(TDH),	defreg(TDLEN),	defreg(TDT),
    defreg(TORH),	defreg(TORL),	defreg(TOTH),	defreg(TOTL),
    defreg(TPR),	defreg(TPT),	defreg(TXDCTL),	defreg(WUFC),
    defreg(RA),		defreg(MTA),	defreg(CRCERRS),defreg(VFTA),
    defreg(VET),        defreg(RDTR),   defreg(RADV),   defreg(TADV),
    defreg(ITR),
};


typedef struct E1000Info {
    const char *name;
    uint16_t   device_id;
    uint8_t    revision;
    uint8_t    io_bar;
    uint16_t   phy_id2;
} E1000Info;


void e1000_class_init(ObjectClass *klass, void *data);

void pci_e1000_realize(PCIDevice *pci_dev, Error **errp);
void pci_e1000_uninit(PCIDevice *dev);
void e1000_reset(void *opaque);
