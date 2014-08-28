/*
 * igb_regs.h
 *
 * Intel 82576 Gigabit Ethernet Adapter
 * (SR/IOV capable PCIe ethernet device) emulation
 *
 * Partly derived from kernel/drivers/net/ethernet/intel/igb/igb.h
 *
 * Copyright (c) 2014 Knut Omang <knut.omang@oracle.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "e1000_regs.h"

/* Experimental igb device (with SR/IOV) IDs for PF and VF: */
#define E1000_DEV_ID_82576                      0x10C9
#define IGB_82576_VF_DEV_ID                     0x10CA

#define I210_I_PHY_ID2 0x0c00
#define IGB_MSIX_BAR 3
#define IGB_TOTAL_VFS 8
#define IGB_VF_OFFSET 0x80
#define IGB_VF_STRIDE 2
#define IGB_CAP_SRIOV_OFFSET 0x160
#define IGB_MSIX_VECTORS_PF 3
#define IGB_MSIX_VECTORS_VF 3
