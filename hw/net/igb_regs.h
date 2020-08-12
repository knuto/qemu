/* SPDX-License-Identifier: GPL-2.0 */
/* Intel(R) Gigabit Ethernet Linux driver
 * Copyright(c) 2007-2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * This is copied + edited from kernel header files in
 * drivers/net/ethernet/intel/igb
 */

#ifndef HW_IGB_REGS_H_
#define HW_IGB_REGS_H_

/* from e1000_hw.h */

#define E1000_DEV_ID_82576                 0x10C9
#define E1000_DEV_ID_82576_FIBER           0x10E6
#define E1000_DEV_ID_82576_SERDES          0x10E7
#define E1000_DEV_ID_82576_QUAD_COPPER             0x10E8
#define E1000_DEV_ID_82576_QUAD_COPPER_ET2 0x1526
#define E1000_DEV_ID_82576_NS                      0x150A
#define E1000_DEV_ID_82576_NS_SERDES               0x1518
#define E1000_DEV_ID_82576_SERDES_QUAD             0x150D

/* from driver igb.h */

#define E1000_PCS_CFG_IGN_SD	1

/* Interrupt defines */
#define IGB_START_ITR		648 /* ~6000 ints/sec */
#define IGB_4K_ITR		980
#define IGB_20K_ITR		196
#define IGB_70K_ITR		56

/* TX/RX descriptor defines */
#define IGB_DEFAULT_TXD		256
#define IGB_DEFAULT_TX_WORK	128
#define IGB_MIN_TXD		80
#define IGB_MAX_TXD		4096

#define IGB_DEFAULT_RXD		256
#define IGB_MIN_RXD		80
#define IGB_MAX_RXD		4096

#define IGB_DEFAULT_ITR		3 /* dynamic */
#define IGB_MAX_ITR_USECS	10000
#define IGB_MIN_ITR_USECS	10
#define NON_Q_VECTORS		1
#define MAX_Q_VECTORS		8
#define MAX_MSIX_ENTRIES	10

/* Transmit and receive queues */
#define IGB_MAX_RX_QUEUES	8
#define IGB_MAX_RX_QUEUES_82575	4
#define IGB_MAX_RX_QUEUES_I211	2
#define IGB_MAX_TX_QUEUES	8
#define IGB_MAX_VF_MC_ENTRIES	30
#define IGB_MAX_VF_FUNCTIONS	8
#define IGB_MAX_VFTA_ENTRIES	128
#define IGB_82576_VF_DEV_ID	0x10CA
#define IGB_I350_VF_DEV_ID	0x1520

/* NVM version defines */
#define IGB_MAJOR_MASK		0xF000
#define IGB_MINOR_MASK		0x0FF0
#define IGB_BUILD_MASK		0x000F
#define IGB_COMB_VER_MASK	0x00FF
#define IGB_MAJOR_SHIFT		12
#define IGB_MINOR_SHIFT		4
#define IGB_COMB_VER_SHFT	8
#define IGB_NVM_VER_INVALID	0xFFFF
#define IGB_ETRACK_SHIFT	16
#define NVM_ETRACK_WORD		0x0042
#define NVM_COMB_VER_OFF	0x0083
#define NVM_COMB_VER_PTR	0x003d

/* Transmit and receive latency (for PTP timestamps) */
#define IGB_I210_TX_LATENCY_10		9542
#define IGB_I210_TX_LATENCY_100		1024
#define IGB_I210_TX_LATENCY_1000	178
#define IGB_I210_RX_LATENCY_10		20662
#define IGB_I210_RX_LATENCY_100		2213
#define IGB_I210_RX_LATENCY_1000	448

/* Number of unicast MAC filters reserved for the PF in the RAR registers */
#define IGB_PF_MAC_FILTERS_RESERVED	3

#define IGB_VF_FLAG_CTS            0x00000001 /* VF is clear to send data */
#define IGB_VF_FLAG_UNI_PROMISC    0x00000002 /* VF has unicast promisc */
#define IGB_VF_FLAG_MULTI_PROMISC  0x00000004 /* VF has multicast promisc */
#define IGB_VF_FLAG_PF_SET_MAC     0x00000008 /* PF has set MAC address */

/* RX descriptor control thresholds.
 * PTHRESH - MAC will consider prefetch if it has fewer than this number of
 *           descriptors available in its onboard memory.
 *           Setting this to 0 disables RX descriptor prefetch.
 * HTHRESH - MAC will only prefetch if there are at least this many descriptors
 *           available in host memory.
 *           If PTHRESH is 0, this should also be 0.
 * WTHRESH - RX descriptor writeback threshold - MAC will delay writing back
 *           descriptors until either it has this many to write back, or the
 *           ITR timer expires.
 */
#define IGB_RX_PTHRESH	((hw->mac.type == e1000_i354) ? 12 : 8)
#define IGB_RX_HTHRESH	8
#define IGB_TX_PTHRESH	((hw->mac.type == e1000_i354) ? 20 : 8)
#define IGB_TX_HTHRESH	1
#define IGB_RX_WTHRESH	((hw->mac.type == e1000_82576 && \
			  (adapter->flags & IGB_FLAG_HAS_MSIX)) ? 1 : 4)
#define IGB_TX_WTHRESH	((hw->mac.type == e1000_82576 && \
			  (adapter->flags & IGB_FLAG_HAS_MSIX)) ? 1 : 16)

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define IGB_RXBUFFER_256	256
#define IGB_RXBUFFER_2048	2048
#define IGB_RXBUFFER_3072	3072
#define IGB_RX_HDR_LEN		IGB_RXBUFFER_256
#define IGB_TS_HDR_LEN		16

/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define IGB_RX_BUFFER_WRITE	16 /* Must be power of 2 */

#define IGB_RX_DMA_ATTR \
	(DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING)

#define AUTO_ALL_MODES		0
#define IGB_EEPROM_APME		0x0400

#ifndef IGB_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define IGB_MASTER_SLAVE	e1000_ms_hw_default
#endif

#define IGB_MNG_VLAN_NONE	-1

enum igb_tx_flags {
	/* cmd_type flags */
	IGB_TX_FLAGS_VLAN	= 0x01,
	IGB_TX_FLAGS_TSO	= 0x02,
	IGB_TX_FLAGS_TSTAMP	= 0x04,

	/* olinfo flags */
	IGB_TX_FLAGS_IPV4	= 0x10,
	IGB_TX_FLAGS_CSUM	= 0x20,
};

/* VLAN info */
#define IGB_TX_FLAGS_VLAN_MASK	0xffff0000
#define IGB_TX_FLAGS_VLAN_SHIFT	16

/* The largest size we can write to the descriptor is 65535.  In order to
 * maintain a power of two alignment we have to limit ourselves to 32K.
 */
#define IGB_MAX_TXD_PWR	15
#define IGB_MAX_DATA_PER_TXD	(1u << IGB_MAX_TXD_PWR)

/* Tx Descriptors needed, worst case */
#define TXD_USE_COUNT(S) DIV_ROUND_UP((S), IGB_MAX_DATA_PER_TXD)
#define DESC_NEEDED (MAX_SKB_FRAGS + 4)

/* EEPROM byte offsets */
#define IGB_SFF_8472_SWAP		0x5C
#define IGB_SFF_8472_COMP		0x5E

/* Bitmasks */
#define IGB_SFF_ADDRESSING_MODE		0x4
#define IGB_SFF_8472_UNSUP		0x00

enum e1000_ring_flags_t {
	IGB_RING_FLAG_RX_3K_BUFFER,
	IGB_RING_FLAG_RX_BUILD_SKB_ENABLED,
	IGB_RING_FLAG_RX_SCTP_CSUM,
	IGB_RING_FLAG_RX_LB_VLAN_BSWAP,
	IGB_RING_FLAG_TX_CTX_IDX,
	IGB_RING_FLAG_TX_DETECT_HANG
};

#define ring_uses_large_buffer(ring) \
	test_bit(IGB_RING_FLAG_RX_3K_BUFFER, &(ring)->flags)
#define set_ring_uses_large_buffer(ring) \
	set_bit(IGB_RING_FLAG_RX_3K_BUFFER, &(ring)->flags)
#define clear_ring_uses_large_buffer(ring) \
	clear_bit(IGB_RING_FLAG_RX_3K_BUFFER, &(ring)->flags)

#define ring_uses_build_skb(ring) \
	test_bit(IGB_RING_FLAG_RX_BUILD_SKB_ENABLED, &(ring)->flags)
#define set_ring_build_skb_enabled(ring) \
	set_bit(IGB_RING_FLAG_RX_BUILD_SKB_ENABLED, &(ring)->flags)
#define clear_ring_build_skb_enabled(ring) \
	clear_bit(IGB_RING_FLAG_RX_BUILD_SKB_ENABLED, &(ring)->flags)

#define IGB_TXD_DCMD (E1000_ADVTXD_DCMD_EOP | E1000_ADVTXD_DCMD_RS)

#define IGB_RX_DESC(R, i)	\
	(&(((union e1000_adv_rx_desc *)((R)->desc))[i]))
#define IGB_TX_DESC(R, i)	\
	(&(((union e1000_adv_tx_desc *)((R)->desc))[i]))
#define IGB_TX_CTXTDESC(R, i)	\
	(&(((struct e1000_adv_tx_context_desc *)((R)->desc))[i]))

/* The number of L2 ether-type filter registers, Index 3 is reserved
 * for PTP 1588 timestamp
 */
#define MAX_ETYPE_FILTER	(4 - 1)
/* ETQF filter list: one static filter per filter consumer. This is
 * to avoid filter collisions later. Add new filters here!!
 *
 * Current filters:		Filter 3
 */
#define IGB_ETQF_FILTER_1588	3

#define IGB_N_EXTTS	2
#define IGB_N_PEROUT	2
#define IGB_N_SDP	4
#define IGB_RETA_SIZE	128

enum igb_filter_match_flags {
	IGB_FILTER_FLAG_ETHER_TYPE = 0x1,
	IGB_FILTER_FLAG_VLAN_TCI   = 0x2,
};

#define IGB_MAX_RXNFC_FILTERS 16

/* flags controlling PTP/1588 function */
#define IGB_PTP_ENABLED		BIT(0)
#define IGB_PTP_OVERFLOW_CHECK	BIT(1)

#define IGB_FLAG_HAS_MSI		BIT(0)
#define IGB_FLAG_DCA_ENABLED		BIT(1)
#define IGB_FLAG_QUAD_PORT_A		BIT(2)
#define IGB_FLAG_QUEUE_PAIRS		BIT(3)
#define IGB_FLAG_DMAC			BIT(4)
#define IGB_FLAG_RSS_FIELD_IPV4_UDP	BIT(6)
#define IGB_FLAG_RSS_FIELD_IPV6_UDP	BIT(7)
#define IGB_FLAG_WOL_SUPPORTED		BIT(8)
#define IGB_FLAG_NEED_LINK_UPDATE	BIT(9)
#define IGB_FLAG_MEDIA_RESET		BIT(10)
#define IGB_FLAG_MAS_CAPABLE		BIT(11)
#define IGB_FLAG_MAS_ENABLE		BIT(12)
#define IGB_FLAG_HAS_MSIX		BIT(13)
#define IGB_FLAG_EEE			BIT(14)
#define IGB_FLAG_VLAN_PROMISC		BIT(15)
#define IGB_FLAG_RX_LEGACY		BIT(16)
#define IGB_FLAG_FQTSS			BIT(17)

/* Media Auto Sense */
#define IGB_MAS_ENABLE_0		0X0001
#define IGB_MAS_ENABLE_1		0X0002
#define IGB_MAS_ENABLE_2		0X0004
#define IGB_MAS_ENABLE_3		0X0008

/* DMA Coalescing defines */
#define IGB_MIN_TXPBSIZE	20408
#define IGB_TX_BUF_4096		4096
#define IGB_DMCTLX_DCFLUSH_DIS	0x80000000  /* Disable DMA Coal Flush */

#define IGB_82576_TSYNC_SHIFT	19
enum e1000_state_t {
	__IGB_TESTING,
	__IGB_RESETTING,
	__IGB_DOWN,
	__IGB_PTP_TX_IN_PROGRESS,
};

#endif
