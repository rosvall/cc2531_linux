// SPDX-FileCopyrightText: 2023 Andreas Sig Rosvall
//
// SPDX-License-Identifier: GPL-3.0-or-later

/*
 * IEEE 802.15.4 driver for CC2531 dongle running WPAN Adapter firmware from
 * https://github.com/rosvall/cc2531_usb_wpan_adapter/
 */

#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/usb.h>
#include <linux/skbuff.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/kref.h>
#include <net/cfg802154.h>
#include <net/mac802154.h>
#include "cc2531_regs.h"
#include "cc2531_csp.h"

MODULE_AUTHOR("Andreas Rosvall <andreas@rosvall.dk>");
MODULE_DESCRIPTION("CC2531-based IEEE 802.15.4 USB Network Adapter Driver");
MODULE_LICENSE("GPL");

#define CC2531_USB_VID 0x1608
#define CC2531_USB_PID 0x154f

static const struct usb_device_id cc2531_device_table[] = {
	{USB_DEVICE_AND_INTERFACE_INFO(
		CC2531_USB_VID,
		CC2531_USB_PID,
		USB_CLASS_VENDOR_SPEC,
		USB_SUBCLASS_VENDOR_SPEC,
		0xff)},
	{},
};
MODULE_DEVICE_TABLE(usb, cc2531_device_table);

/*
 * Custom control requests
 */
enum cc2531_usb_request {
	CC2531_USB_REQ_XDATA_READ   = 0,
	CC2531_USB_REQ_XDATA_WRITE  = 1,
	CC2531_USB_REQ_FIFO_READ    = 2,
	CC2531_USB_REQ_FIFO_WRITE   = 3,
	CC2531_USB_REQ_TX           = 4,
	CC2531_USB_REQ_SET_CSMA     = 5,
};
enum cc2531_usb_request_type {
	USB_RT_VENDOR_IN  = USB_DIR_IN  | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
	USB_RT_VENDOR_OUT = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
};

/*
 * Special string descriptors
 */
enum cc2531_string_desc {
	CC2531_STRING_DESC_VERSION = 0x80, /* Firmware version string */
};

/*
 * HW Constants
 */
#define CC2531_CHAN_MIN 11
#define CC2531_CHAN_MAX 31
/*
 * Ref.: http://www.ti.com/lit/pdf/SWRU191F p.264 CCACTRL1 (0x6197) - Other CCA Options
 */
enum cc2531_cca_mode {
	CC2531_CCA_MODE_DISABLED    = 0,
	CC2531_CCA_MODE_RSSI        = 1,
	CC2531_CCA_MODE_RX          = 2,
	CC2531_CCA_MODE_RSSI_AND_RX = 3,
};

#define CC2531_RX_URB_COUNT  4
#define CC2531_INT_URB_COUNT 4
#define CC2531_RX_SIZE 128

struct cc2531 {
	struct kref          kref;
	struct delayed_work  resubmit_delayed;
	struct work_struct   tx_work;
	struct usb_anchor    idle_urbs,
	                     submitted;
	struct completion    status_received,
	                     got_ack;
	struct ieee802154_hw *hw;
	struct usb_interface *intf;
	struct device        *dev;
	struct usb_device    *usb;
	struct sk_buff       *tx_skb;
	spinlock_t           lock;
	bool                 lbt;
	unsigned             frame_retries;
	atomic_t             tx_status,
	                     expected_ack_seq;
};

#define CC2531_FREQCTRL_TO_CHANNEL(_v)   (11 + ((_v) - 11)/5)
#define CC2531_CHANNEL_TO_FREQCTRL(_ch)  (11 + 5*((_ch) - 11))

/*
 * Ref.: http://www.ti.com/lit/pdf/SWRS086 Table 1. Recommended Output Power
 * Settings. These values should have some offset added depending on board.
 */
static const s32 cc2531_powers[] = {
	/* mBm  TX_POWER reg */
	-2200, /*0x05*/
	-2000, /*0x15*/
	-1800, /*0x25*/
	-1600, /*0x35*/
	-1400, /*0x45*/
	-1200, /*0x55*/
	-1000, /*0x65*/
	-800, /*0x75*/
	-600, /*0x85*/
	-400, /*0x95*/
	-300, /*0xA5*/
	-150, /*0xB5*/
	-50, /*0xC5*/
	100, /*0xD5*/
	250, /*0xE5*/
	450, /*0xF5*/
};

/*
 * Clear channel assessment energy levels in mBm.
 * The chip will accept anything from -204dBm up to 51dBm.
 * (-128-76dBm to 127-76dBm)
 * The noise floor is around -95dBm though, so lower values make little sense.
 */
static const s32 cc2531_ed_levels[] = {
	-10000, -9900, -9800, -9700, -9600, -9500, -9400, -9300, -9200, -9100, -9000, -8900, -8800,
	-8700,	-8600, -8500, -8400, -8300, -8200, -8100, -8000, -7900, -7800, -7700, -7600, -7500,
	-7400,	-7300, -7200, -7100, -7000, -6900, -6800, -6700, -6600, -6500, -6400, -6300, -6200,
	-6100,	-6000, -5900, -5800, -5700, -5600, -5500, -5400, -5300, -5200, -5100, -5000, -4900,
	-4800,	-4700, -4600, -4500, -4400, -4300, -4200, -4100, -4000,
};
#define CC2531_DBM_OFFSET	          -76
#define CC2531_MBM_TO_HWLVL(_mbm)     ((_mbm) / 100 - CC2531_DBM_OFFSET)
#define CC2531_HWLVL_TO_MBM(_hw_rssi) (100 * ((s8)(_hw_rssi) + CC2531_DBM_OFFSET))

static void cc2531_set_hw_caps(struct ieee802154_hw *hw)
{
	hw->flags                = IEEE802154_HW_OMIT_CKSUM |
	                           IEEE802154_HW_AFILT |
	                           IEEE802154_HW_PROMISCUOUS |
	                           IEEE802154_HW_FRAME_RETRIES |
	                           IEEE802154_HW_CSMA_PARAMS;

	hw->phy->flags           = WPAN_PHY_FLAG_TXPOWER |
	                           WPAN_PHY_FLAG_CCA_ED_LEVEL |
	                           WPAN_PHY_FLAG_CCA_MODE;

	hw->phy->symbol_duration = 16000;

	hw->phy->supported = (struct wpan_phy_supported){
		.iftypes             = BIT(NL802154_IFTYPE_NODE) |
                               BIT(NL802154_IFTYPE_MONITOR) |
                               BIT(NL802154_IFTYPE_COORD),

		.cca_modes           = BIT(NL802154_CCA_ENERGY) |
                               BIT(NL802154_CCA_CARRIER) |
                               BIT(NL802154_CCA_ENERGY_CARRIER) |
                               BIT(NL802154_CCA_ALOHA),

		.cca_opts            = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND),

		.min_frame_retries   = 0,
		.max_frame_retries   = 32,

		.min_minbe           = 0,
		.max_minbe           = 7,

		.min_maxbe           = 0,
		.max_maxbe           = 7,

		.min_csma_backoffs   = 0,
		.max_csma_backoffs   = 32,

		.lbt                 = NL802154_SUPPORTED_BOOL_TRUE,

		.channels[0]         = GENMASK(CC2531_CHAN_MAX, CC2531_CHAN_MIN),

		.tx_powers           = cc2531_powers,
		.tx_powers_size      = ARRAY_SIZE(cc2531_powers),

		.cca_ed_levels       = cc2531_ed_levels,
		.cca_ed_levels_size  = ARRAY_SIZE(cc2531_ed_levels),
	};
}

static void cc2531_delete(struct kref *kref)
{
	struct cc2531 *cc = container_of(kref, struct cc2531, kref);
	struct urb *urb;
	struct device *dev = cc->dev;

	dev_dbg(dev, "%s", __func__);

	while ((urb = usb_get_from_anchor(&cc->idle_urbs))) {
		usb_free_coherent(cc->usb, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
		usb_put_urb(urb);

		dev_dbg(dev, "freed urb %p", urb);
	}

	usb_put_intf(cc->intf);
	usb_put_dev(cc->usb);

	ieee802154_free_hw(cc->hw);

	dev_dbg(dev, "%s done", __func__);
}

static int cc2531_control_msg_recv(struct cc2531 *cc, enum cc2531_usb_request req, u16 value, u16 index, void *data, u16 size)
{
	int err;

	err = usb_autopm_get_interface(cc->intf);
	if (err) {
		dev_err(cc->dev, "usb_autopm_get_interface failed: %d", err);
		return err;
	}

	err = usb_control_msg_recv(cc->usb, 0, req, USB_RT_VENDOR_IN, value, index, data, size, USB_CTRL_GET_TIMEOUT, GFP_KERNEL);
	if (err)
		dev_err(cc->dev, "usb_control_msg_recv error %d", err);

	usb_autopm_put_interface(cc->intf);

	return err;
}

static int cc2531_control_msg_send(struct cc2531 *cc, enum cc2531_usb_request req, u16 value, u16 index, const void *data, u16 size)
{
	int err;

	err = usb_autopm_get_interface(cc->intf);
	if (err) {
		dev_err(cc->dev, "usb_autopm_get_interface failed: %d", err);
		return err;
	}

	err = usb_control_msg_send(cc->usb, 0, req, USB_RT_VENDOR_OUT, value, index, data, size, USB_CTRL_GET_TIMEOUT, GFP_KERNEL);
	if (err)
		dev_err(cc->dev, "usb_control_msg_send error %d", err);

	usb_autopm_put_interface(cc->intf);

	return err;
}

inline int cc2531_write(struct cc2531 *cc, enum cc2531_reg_addr addr, const void *data, u16 size)
{
	return cc2531_control_msg_send(cc, CC2531_USB_REQ_XDATA_WRITE, addr, 0, data, size);
}

inline int cc2531_write_byte(struct cc2531 *cc, enum cc2531_reg_addr addr, u8 byte)
{
	return cc2531_write(cc, addr, &byte, 1);
}

inline int csp_imm_strobe(struct cc2531 *cc, enum csp_cmd_strobe strobe)
{
	u8 imm_strobe = CSP_IMM_CMD_STROBE(strobe);
	return cc2531_write_byte(cc, CC2531_REG_RFST_ADDR, imm_strobe);
}

inline int cc2531_read(struct cc2531 *cc, enum cc2531_reg_addr addr, void *data, u16 size)
{
	return cc2531_control_msg_recv(cc, CC2531_USB_REQ_XDATA_READ, addr, 0, data, size);
}

inline int cc2531_read_byte(struct cc2531 *cc, enum cc2531_reg_addr addr, u8 *byte)
{
	return cc2531_read(cc, addr, byte, 1);
}

static int cc2531_write_reg_bits(struct cc2531 *cc, enum cc2531_reg_addr addr, u8 msb, u8 lsb,
				 u8 val)
{
	int err;
	u8 mask = GENMASK(msb, lsb);
	u8 tmp;

	val <<= lsb;
	val &= mask;

	err = cc2531_read_byte(cc, addr, &tmp);
	if (err)
		return err;

	if ((tmp & mask) == val)
		return 0;

	tmp &= ~mask;
	tmp |= val;

	return cc2531_write_byte(cc, addr, tmp);
}

#define regbits_write(_dev, _name, _var)                                           \
	cc2531_write_reg_bits(_dev, CC2531_REG_ADDR(_name), CC2531_REG_MSB(_name), \
			      CC2531_REG_LSB(_name), _var)

static int cc2531_read_reg_bits(struct cc2531 *cc, unsigned int addr, u8 msb, u8 lsb, u8 *val)
{
	int err;
	err = cc2531_read_byte(cc, addr, val);

	*val &= GENMASK(msb, lsb);
	*val >>= lsb;

	return err;
}

#define regbits_read(_dev, _name, _var)                                           \
	cc2531_read_reg_bits(_dev, CC2531_REG_ADDR(_name), CC2531_REG_MSB(_name), \
			     CC2531_REG_LSB(_name), _var)

static int cc2531_send_pkt(struct cc2531 *cc, void *data, unsigned len)
{
	u8 mode = 0;

	dev_dbg(cc->dev, "sending pkt\n");

	return cc2531_control_msg_send(cc, CC2531_USB_REQ_TX, mode, 0, data, len);
}

static u8 cc2531_mfr_to_lqi(u8 *mfr)
{
	const int corr_min = 60, corr_max = 110, rssi_min = -30, rssi_max = 60;
	int lqi, corr, rssi, crc_ok;

	/* RSSI comes as 8 bit signed int */
	rssi = (int)(s8)mfr[0];
	crc_ok = mfr[1] & 0x80;
	corr = mfr[1] & 0x7f;

	if (!crc_ok)
		return 0;

	corr = 255 * (corr - corr_min) / (corr_max - corr_min);
	corr = clamp(corr, 0, 255);

	rssi = 255 * (rssi - rssi_min) / (rssi_max - rssi_min);
	rssi = clamp(rssi, 0, 255);

	lqi = corr * 3 / 4 + rssi * 1 / 4;

	return lqi;
}

static inline bool ieee802154_is_ack(__le16 fc)
{
	return IEEE802154_FC_TYPE(fc) == cpu_to_le16(IEEE802154_FC_TYPE_ACK);
}

static void cc2531_pkt_received(struct cc2531 *cc, u8 *data, u32 len)
{
	struct sk_buff *skb;
	u8 msdu_len = len - IEEE802154_MFR_SIZE;
	u8 *mac_footer = data + msdu_len;
	u8 lqi;
	__le16 fc;

	dev_dbg(cc->dev, "pkt_received %p %u", data, len);

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_warn(cc->dev, "Received pkt with invalid length: %u\n", len);
		goto drop_pkt;
	}

	lqi = cc2531_mfr_to_lqi(mac_footer);
	if (!lqi) {
		dev_warn(cc->dev, "Received pkt with invalid crc\n");
		goto drop_pkt;
	}

	fc = __get_unaligned_t(__le16, data);
	if (ieee802154_is_ack(fc)) {
		int seq = data[IEEE802154_FC_LEN];
		int expected;
		unsigned long flags;

		spin_lock_irqsave(&cc->lock, flags);
		expected = atomic_read(&cc->expected_ack_seq);
		spin_unlock_irqrestore(&cc->lock, flags);

		dev_dbg(cc->dev, "got an ack. seq: %u", seq);
		if (seq == expected) {
			dev_dbg(cc->dev, "it's the one we're waiting for!");
			complete(&cc->got_ack);
		}
	}

	skb = alloc_skb(msdu_len, GFP_ATOMIC);
	if (!skb) {
		dev_err(cc->dev, "Could not alloc skb for received packet\n");
		goto drop_pkt;
	}

	skb_put_data(skb, data, msdu_len);

	ieee802154_rx_irqsafe(cc->hw, skb, lqi);
	return;

drop_pkt:
	print_hex_dump(KERN_DEBUG, "pkt: ", DUMP_PREFIX_NONE, 16, 1, data, len, false);
}

static void cc2531_int_received(struct cc2531 *cc, u8 status)
{
	dev_dbg(cc->dev, "int status: %u", status);

	if (completion_done(&cc->status_received)) {
		WARN_ON(1);
		return;
	}

	atomic_set(&cc->tx_status, status);
	complete(&cc->status_received);
}

static int cc2531_transmit_once(struct cc2531 *cc, struct sk_buff *skb)
{
	const unsigned long timeout = msecs_to_jiffies(5000);
	int err;
	int status;
	
	WARN_ON(swq_has_sleeper(&cc->status_received.wait));

	reinit_completion(&cc->status_received);

	atomic_set(&cc->tx_status, -ETIMEDOUT);

	err = cc2531_send_pkt(cc, skb->data, skb->len);
	if (err) {
		dev_err(cc->dev, "cc2531_send_pkt failed: %d", err);
		return err;
	}

	// Wait for status_urb to get tx status
	if (!wait_for_completion_timeout(&cc->status_received, timeout)) {
		dev_err(cc->dev, "Timed out waiting for status urb");
	}

	status = atomic_read(&cc->tx_status);
	if (!status)
		dev_dbg(cc->dev, "transmitted ok");

	return status;
}

static int cc2531_transmit_with_retries(struct cc2531 *cc, struct sk_buff *skb)
{
	const unsigned long timeout = msecs_to_jiffies(20);
	unsigned remaining_frame_retries = cc->frame_retries;
	u8 seq = skb->data[IEEE802154_FC_LEN];
	int status;
	unsigned long flags;

	dev_dbg(cc->dev, "sending with ackreq. seq: %u", seq);


	spin_lock_irqsave(&cc->lock, flags);
	atomic_set(&cc->expected_ack_seq, seq);
	spin_unlock_irqrestore(&cc->lock, flags);

	WARN_ON(swq_has_sleeper(&cc->got_ack.wait));

	reinit_completion(&cc->got_ack);

	do {
		status = cc2531_transmit_once(cc, skb);

		if (status < 0)
			goto end;
		
		if (status > 0) {
			dev_dbg(cc->dev, "waiting for ack: got tx status: %d", status);
			continue;
		}

		if (wait_for_completion_timeout(&cc->got_ack, timeout)) {
			dev_dbg(cc->dev, "Acked");
			goto end;
		} else {
			dev_dbg(cc->dev, "Timed out waiting for ACK");
		}

		if (remaining_frame_retries)
			dev_dbg(cc->dev, "retransmitting...");

	} while (remaining_frame_retries--);

	dev_info(cc->dev, "Unacked: seq no %u", seq);
	if (!status)
		status = IEEE802154_NO_ACK;

end:
	spin_lock_irqsave(&cc->lock, flags);
	atomic_set(&cc->expected_ack_seq, -1);
	spin_unlock_irqrestore(&cc->lock, flags);

	return status;
}

static void cc2531_do_tx_work(struct work_struct *work)
{
	struct cc2531 *cc = container_of(work, struct cc2531, tx_work);
	int tx_status = IEEE802154_SYSTEM_ERROR;
	struct sk_buff *skb;
	unsigned long flags;
	__le16 fc;


	spin_lock_irqsave(&cc->lock, flags);
	skb = READ_ONCE(cc->tx_skb);
	spin_unlock_irqrestore(&cc->lock, flags);

	dev_dbg(cc->dev, "do_tx_work tx_skb: %p", skb);

	WARN_ON(!skb);
	if (!skb)
		return;

	fc = __get_unaligned_t(__le16, skb->data);
	if (ieee802154_is_ackreq(fc)) {
		tx_status = cc2531_transmit_with_retries(cc, skb);
	} else {
		tx_status = cc2531_transmit_once(cc, skb);
	}

	spin_lock_irqsave(&cc->lock, flags);
	WRITE_ONCE(cc->tx_skb, NULL);
	spin_unlock_irqrestore(&cc->lock, flags);

	if (tx_status) {
		dev_dbg(cc->dev, "bad tx_status: %d", tx_status);
		ieee802154_xmit_error(cc->hw, skb, tx_status);
	} else {
		dev_dbg(cc->dev, "tx complete");
		ieee802154_xmit_complete(cc->hw, skb, false);
	}
}

static int cc2531_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	unsigned long flags;
	struct cc2531 *cc = hw->priv;

	dev_dbg(cc->dev, "xmit tx_skb: %p\n", skb);

	spin_lock_irqsave(&cc->lock, flags);
	WARN_ON(READ_ONCE(cc->tx_skb));
	WRITE_ONCE(cc->tx_skb, skb);
	spin_unlock_irqrestore(&cc->lock, flags);

	schedule_work(&cc->tx_work);

	return 0;
}

static int cc2531_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct cc2531 *cc = hw->priv;

	WARN_ON(level == NULL);

	dev_dbg(cc->dev, "%s\n", __func__);

	return cc2531_read(cc, CC2531_REG_RSSI_ADDR, level, sizeof(*level));
}

static int cc2531_set_hw_addr_filt(struct ieee802154_hw *hw, struct ieee802154_hw_addr_filt *filt,
				   unsigned long changed)
{
	struct cc2531 *cc = hw->priv;
	int err;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		dev_dbg(cc->dev, "hw filter: Address: %#06x\n", le16_to_cpu(filt->short_addr));

		err = cc2531_write(cc, CC2531_REG_SHORT_ADDR_ADDR, &filt->short_addr,
				   sizeof(filt->short_addr));
		if (err)
			return err;
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		dev_dbg(cc->dev, "hw filter: PAN: %#06x\n", le16_to_cpu(filt->pan_id));

		err = cc2531_write(cc, CC2531_REG_PAN_ID_ADDR, &filt->pan_id, sizeof(filt->pan_id));
		if (err)
			return err;
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		dev_dbg(cc->dev, "hw filter: Extended address: %016llx\n",
			le64_to_cpu(filt->ieee_addr));

		err = cc2531_write(cc, CC2531_REG_EXT_ADD_ADDR, &filt->ieee_addr,
				   sizeof(filt->ieee_addr));
		if (err)
			return err;
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		dev_dbg(cc->dev, "hw filter: Is PAN coordinator: %d\n", filt->pan_coord);

		err = regbits_write(cc, PAN_COORDINATOR, filt->pan_coord);
		if (err)
			return err;
	}

	return 0;
}

static int cc2531_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct cc2531 *cc = hw->priv;

	for (int i = 0; i < ARRAY_SIZE(cc2531_powers); i++) {
		if (cc2531_powers[i] == mbm) {
			u8 reg_value = (i << 4) | 0x05;
			dev_dbg(cc->dev, "setting tx power: %d mBm\n", mbm);
			return cc2531_write_byte(cc, CC2531_REG_TXPOWER_ADDR, reg_value);
		}
	}

	return -EINVAL;
}

static int cc2531_set_cca_mode(struct ieee802154_hw *hw, const struct wpan_phy_cca *cca)
{
	struct cc2531 *cc = hw->priv;
	u8 mode;

	dev_dbg(cc->dev, "setting cca mode: %d\n", cca->mode);

	/* mapping 802.15.4 to driver spec */
	switch (cca->mode) {
	case NL802154_CCA_ENERGY:
		mode = CC2531_CCA_MODE_RSSI;
		break;
	case NL802154_CCA_CARRIER:
		mode = CC2531_CCA_MODE_RX;
		break;
	case NL802154_CCA_ENERGY_CARRIER:
		mode = CC2531_CCA_MODE_RSSI_AND_RX;
		break;
	case NL802154_CCA_ALOHA:
		mode = CC2531_CCA_MODE_DISABLED;
		break;
	default:
		return -EINVAL;
	}

	return regbits_write(cc, CCA_MODE, mode);
}

static int cc2531_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct cc2531 *cc = hw->priv;
	s32 value;
	u8 reg_value;

	value = CC2531_MBM_TO_HWLVL(mbm);

	if (value < S8_MIN || value > S8_MAX)
		return -EINVAL;

	dev_dbg(cc->dev, "setting cca energy: %d mBm\n", mbm);

	reg_value = (u8)value;

	return regbits_write(cc, CCA_THR, reg_value);
}

static int cc2531_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct cc2531 *cc = hw->priv;
	u8 reg_value;

	if (page != 0)
		return -EINVAL;

	dev_dbg(cc->dev, "setting ch: %d\n", channel);

	reg_value = CC2531_CHANNEL_TO_FREQCTRL(channel);
	return cc2531_write_byte(cc, CC2531_REG_FREQCTRL_ADDR, reg_value);
}

static int cc2531_set_lbt(struct ieee802154_hw *hw, bool on)
{
	struct cc2531 *cc = hw->priv;

	dev_dbg(cc->dev, "setting lbt %s\n", on ? "on" : "off");

	cc->lbt = on;

	return 0;
}

static int cc2531_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct cc2531 *cc = hw->priv;

	dev_dbg(cc->dev, "promiscous mode: %d\n", on);

	return regbits_write(cc, FRAME_FILTER_EN, !on);
}

static int cc2531_set_frame_retries(struct ieee802154_hw *hw, const s8 retries)
{
	struct cc2531 *cc = hw->priv;

	if (retries < 0)
		return -EINVAL;

	dev_dbg(cc->dev, "setting frame retries: %u\n", retries);

	cc->frame_retries = (u8)retries;

	return 0;
}

static int cc2531_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be, u8 retries)
{
	struct cc2531 *cc = hw->priv;
	u16 packed_csma_settings = (retries<<8) | (max_be << 4) | min_be;

	dev_dbg(cc->dev, "CSMA params: min_be: %u max_be: %u be_retries: %u", min_be, max_be, retries);

	return cc2531_control_msg_send(cc, CC2531_USB_REQ_SET_CSMA, packed_csma_settings, 0, NULL, 0);
}

static int cc2531_start(struct ieee802154_hw *hw)
{
	struct cc2531 *cc = hw->priv;
	int err;

	err = usb_autopm_get_interface(cc->intf);
	if (err)
		return err;

	kref_get(&cc->kref);

	dev_dbg(cc->dev, "START\n");

	schedule_delayed_work(&cc->resubmit_delayed, 0);

	return csp_imm_strobe(cc, CSP_CMD_RXON);
}

static void cc2531_stop(struct ieee802154_hw *hw)
{
	struct cc2531 *cc = hw->priv;

	dev_dbg(cc->dev, "STOP\n");

	csp_imm_strobe(cc, CSP_CMD_RFOFF);

	usb_kill_anchored_urbs(&cc->submitted);

	kref_put(&cc->kref, cc2531_delete);

	usb_autopm_put_interface(cc->intf);
}

static const struct ieee802154_ops cc2531_ieee802154_ops = {
	.owner                = THIS_MODULE,
	.xmit_async           = cc2531_xmit,
	.ed                   = cc2531_ed,
	.set_channel          = cc2531_set_channel,
	.start                = cc2531_start,
	.stop                 = cc2531_stop,
	.set_hw_addr_filt     = cc2531_set_hw_addr_filt,
	.set_txpower          = cc2531_set_txpower,
	.set_lbt              = cc2531_set_lbt,
	.set_cca_mode         = cc2531_set_cca_mode,
	.set_cca_ed_level     = cc2531_set_cca_ed_level,
	.set_promiscuous_mode = cc2531_set_promiscuous_mode,
	.set_frame_retries    = cc2531_set_frame_retries,
	.set_csma_params      = cc2531_set_csma_params,
};


static int cc2531_read_perm_extaddr(struct cc2531 *cc)
{
	int err;
	__le64 *addr = &cc->hw->phy->perm_extended_addr;

	err = cc2531_read(cc, CC2531_REG_IEEE_PERM_ADDR_ADDR, addr, sizeof(*addr));
	if (err)
		return err;

	dev_info(cc->dev, "read permanent extended address: %016llx", le64_to_cpu(*addr));

	return err;
}

static int cc2531_read_version_str(struct cc2531 *cc)
{
	char version_str[256] = { 0 };
	int len = usb_string(cc->usb, CC2531_STRING_DESC_VERSION, version_str, sizeof(version_str));
	if (len < 0)
		return len;
	dev_info(cc->dev, "CC2531 firmware version: %s", version_str);
	return 0;
}

static int cc2531_configure_chip(struct cc2531 *cc)
{
	struct ieee802154_hw *hw = cc->hw;
	struct wpan_phy *phy = hw->phy;
	u8 regval;
	int err;
	// Read regs

	err = cc2531_read_perm_extaddr(cc);
	if (err)
		return err;

	err = cc2531_read_version_str(cc);
	if (err)
		return err;

	err = cc2531_read_byte(cc, CC2531_REG_FREQCTRL_ADDR, &regval);
	if (err)
		return err;

	phy->current_page = 0;
	phy->current_channel = CC2531_FREQCTRL_TO_CHANNEL(regval);
	dev_dbg(cc->dev, "current channel: %d\n", phy->current_channel);

	err = cc2531_read_byte(cc, CC2531_REG_TXPOWER_ADDR, &regval);
	if (err)
		return err;
	phy->transmit_power = phy->supported.tx_powers[regval >> 4];
	dev_dbg(cc->dev, "current TX power: %d mBm\n", phy->transmit_power);

	err = regbits_read(cc, CCA_THR, &regval);
	if (err)
		return err;
	phy->cca_ed_level = CC2531_HWLVL_TO_MBM(regval);
	dev_dbg(cc->dev, "current CCA energy: %d mBm\n", phy->cca_ed_level);

	err = regbits_read(cc, CCA_MODE, &regval);
	if (err)
		return err;
	switch (regval) {
	case CC2531_CCA_MODE_RSSI:
		hw->phy->cca.mode = NL802154_CCA_ENERGY;
		break;
	case CC2531_CCA_MODE_RX:
		hw->phy->cca.mode = NL802154_CCA_CARRIER;
		break;
	case CC2531_CCA_MODE_RSSI_AND_RX:
		hw->phy->cca.mode = NL802154_CCA_ENERGY_CARRIER;
		break;
	case CC2531_CCA_MODE_DISABLED:
		hw->phy->cca.mode = NL802154_CCA_ALOHA;
		break;
	}
	dev_dbg(cc->dev, "current CCA mode: %d\n", phy->cca.mode);

	return 0;
}

static int cc2531_resubmit_urb(struct cc2531 *cc, struct urb *urb, gfp_t flags)
{
	const unsigned long resubmit_delay = msecs_to_jiffies(1000);
	int err = usb_submit_urb(urb, flags);
	// dev_dbg(cc->dev, "%s %p", __func__, urb);
	if (err) {
		dev_err(cc->dev, "resubmit failed: %d\n", err);
		usb_anchor_urb(urb, &cc->idle_urbs);
		usb_put_urb(urb);
		schedule_delayed_work(&cc->resubmit_delayed, resubmit_delay);
	} else {
		usb_anchor_urb(urb, &cc->submitted);
	}

	return err;
}

static void cc2531_resubmit_idle_urbs(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct cc2531 *cc = container_of(dw, struct cc2531, resubmit_delayed);
	struct urb *urb;

	dev_dbg(cc->dev, "resubmitting later...");

	while ((urb = usb_get_from_anchor(&cc->idle_urbs))) {
		int err = cc2531_resubmit_urb(cc, urb, GFP_KERNEL);
		if (err)
			return;
	}
}

static void cc2531_urb_done(struct urb *urb)
{
	struct cc2531 *cc = urb->context;

	switch (urb->status) {
	case 0:
		switch (usb_endpoint_type(&urb->ep->desc)) {
		case USB_ENDPOINT_XFER_BULK:
			cc2531_pkt_received(cc, urb->transfer_buffer, urb->actual_length);
			break;
		case USB_ENDPOINT_XFER_INT:
			cc2531_int_received(cc, *(u8 *)urb->transfer_buffer);
			break;
		}
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		dev_dbg(cc->dev, "urb dead: %d", urb->status);
		usb_anchor_urb(urb, &cc->idle_urbs);
		usb_put_urb(urb);
		return;
	default:
		dev_err(cc->dev, "urb status = %d\n", urb->status);
	}

	cc2531_resubmit_urb(cc, urb, GFP_ATOMIC);
}

static int cc2531_create_idle_in_urb(struct cc2531 *cc, struct usb_endpoint_descriptor *ep, unsigned bufsize)
{
	unsigned pipe;
	struct urb *urb;
	void *buf;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

	buf = usb_alloc_coherent(cc->usb, bufsize, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	switch (usb_endpoint_type(ep)) {
	case USB_ENDPOINT_XFER_BULK:
		pipe = usb_rcvbulkpipe(cc->usb, ep->bEndpointAddress);
		usb_fill_bulk_urb(urb, cc->usb, pipe, buf, bufsize, cc2531_urb_done, cc);
		break;
	case USB_ENDPOINT_XFER_INT:
	 	pipe = usb_rcvintpipe(cc->usb, ep->bEndpointAddress);
		usb_fill_int_urb(urb, cc->usb, pipe, buf, bufsize, cc2531_urb_done, cc, ep->bInterval);
		break;
	}

	usb_anchor_urb(urb, &cc->idle_urbs);
	usb_put_urb(urb);

	return 0;
}

static int cc2531_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct ieee802154_hw *hw;
	struct cc2531 *cc;
	struct usb_endpoint_descriptor *recv_ep, *intr_ep;
	int err;

	/* ieee802154_alloc_hw returns zeroed memory */
	hw = ieee802154_alloc_hw(sizeof(*cc), &cc2531_ieee802154_ops);
	if (!hw)
		return -ENOMEM;

	cc = hw->priv;
	cc->hw = hw;
	cc->usb = usb_get_dev(interface_to_usbdev(intf));
	cc->intf = usb_get_intf(intf);
	cc->dev = &intf->dev;

	usb_set_intfdata(intf, cc);
	hw->parent = cc->dev;

	kref_init(&cc->kref);
	spin_lock_init(&cc->lock);
	INIT_DELAYED_WORK(&cc->resubmit_delayed, cc2531_resubmit_idle_urbs);
	INIT_WORK(&cc->tx_work, cc2531_do_tx_work);
	init_completion(&cc->status_received);
	init_completion(&cc->got_ack);
	init_usb_anchor(&cc->idle_urbs);
	init_usb_anchor(&cc->submitted);

	err = usb_find_common_endpoints(intf->cur_altsetting, &recv_ep, NULL, &intr_ep, NULL);
	if (err) {
		dev_err(cc->dev, "Failed to find required bulk in and intr in endpoints");
		goto error;
	}

	for (int i = 0; i < CC2531_RX_URB_COUNT; i++) {
		err = cc2531_create_idle_in_urb(cc, recv_ep, CC2531_RX_SIZE);
		if (err) {
			dev_err(cc->dev, "Could not create RX URB");
			goto error;
		}
	}

	for (int i = 0; i < CC2531_INT_URB_COUNT; i++) {
		err = cc2531_create_idle_in_urb(cc, intr_ep, intr_ep->wMaxPacketSize);
		if (err) {
			dev_err(cc->dev, "Could not create INT URB");
			goto error;
		}
	}

	cc2531_set_hw_caps(hw);

	err = cc2531_configure_chip(cc);
	if (err) {
		dev_err(cc->dev, "cc2531_configure_chip failed: %d", err);
		goto error;
	}

	err = ieee802154_register_hw(hw);
	if (err) {
		dev_err(cc->dev, "ieee802154_register_hw failed: %d", err);
		goto error;
	}

	return 0;

error:
	dev_err(cc->dev, "Initialization failed: %d\n", err);
	kref_put(&cc->kref, cc2531_delete);
	return err;
}

static void cc2531_disconnect(struct usb_interface *intf)
{
	struct cc2531 *cc = usb_get_intfdata(intf);

	cancel_delayed_work_sync(&cc->resubmit_delayed);
	cancel_work_sync(&cc->tx_work);

	ieee802154_unregister_hw(cc->hw);

	kref_put(&cc->kref, cc2531_delete);

	dev_dbg(&intf->dev, "%s done\n", __func__);
}

static int cc2531_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct cc2531 *cc = usb_get_intfdata(intf);
	dev_dbg(cc->dev, "suspend: %x", message.event);
	return 0;
}

static int cc2531_resume(struct usb_interface *intf)
{
	struct cc2531 *cc = usb_get_intfdata(intf);
	dev_dbg(cc->dev, "resume");
	return 0;
}

/*
 * Expose registers in sysfs for fun and experimentation.
 * They probably belong in debugfs, but this is easier.
 */

struct reg_attr {
	struct device_attribute attr;
	enum cc2531_reg_addr reg;
	int msb;
	int lsb;
};

static ssize_t cc2531_show_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct reg_attr *reg_attr = container_of(attr, struct reg_attr, attr);
	struct cc2531 *cc = dev_get_drvdata(dev);
	int nbits = 1 + reg_attr->msb - reg_attr->lsb;
	int err;

	if (nbits < 8) {
		u8 value;
		err = cc2531_read_reg_bits(cc, reg_attr->reg, reg_attr->msb, reg_attr->lsb, &value);
		if (err)
			return err;
		return sysfs_emit(buf, "%#x\n", value);
	} else {
		int n = nbits / 8;
		u8 raw[CC2531_MAX_REG_LEN];
		int len;

		err = cc2531_read(cc, reg_attr->reg, raw, n);
		if (err)
			return err;

		len = sysfs_emit(buf, "0x");

		while (n--)
			len += sysfs_emit_at(buf, len, "%02x", raw[n]);

		len += sysfs_emit_at(buf, len, "\n");

		return len;
	}
}

static ssize_t cc2531_store_attr(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct reg_attr *reg_attr = container_of(attr, struct reg_attr, attr);
	struct cc2531 *cc = dev_get_drvdata(dev);
	int err;
	int nbits = 1 + reg_attr->msb - reg_attr->lsb;
	u64 val;

	err = kstrtou64(buf, 0, &val);
	if (err)
		return err;

	if (val >= (1U << nbits))
		return -EINVAL;

	if (nbits < 8)
		err = cc2531_write_reg_bits(cc, reg_attr->reg, reg_attr->msb, reg_attr->lsb, val);
	else
		err = cc2531_write(cc, reg_attr->reg, &val, nbits / 8);

	if (err)
		return err;

	return count;
}

#define DEFINE_REG_ATTR(_addr, _lsb, _msb, _name)                         \
	static struct reg_attr _name##_attr = {                               \
		.attr = __ATTR(_name, 0600, cc2531_show_attr, cc2531_store_attr), \
		.reg  = _addr,                                                    \
		.msb  = _msb,                                                     \
		.lsb  = _lsb,                                                     \
	};

FOR_EACH_CC2531_REG(DEFINE_REG_ATTR)

#define LIST_REG_ATTR(_addr, _lowbit, _highbit, _name) &_name##_attr.attr.attr,

static struct attribute *cc2531_attrs[] = {
	FOR_EACH_CC2531_REG(LIST_REG_ATTR)
	NULL,
};

static const struct attribute_group cc2531_group = {
	.name  = "cc2531_reg",
	.attrs = cc2531_attrs,
};

static const struct attribute_group *cc2531_groups[] = {
	&cc2531_group,
	NULL,
};

static struct usb_driver cc2531_driver = {
	.name                 = "cc2531",
	.id_table             = cc2531_device_table,
	.dev_groups           = cc2531_groups,
	.supports_autosuspend = 1,

	.probe                = cc2531_probe,
	.disconnect           = cc2531_disconnect,
	.suspend              = cc2531_suspend,
	.resume               = cc2531_resume,
};

module_usb_driver(cc2531_driver);
