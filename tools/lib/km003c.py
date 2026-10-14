"""POWER-Z KM003C protocol pieces the meter emulates for PD capture: the
GetData/PutData framing, the PdPacket payload (measurement preamble +
connection / wrapped-PD-message events), and a printer that decodes a
payload's events with pd_decode. Firmware side: rust/src/powerz.rs and
rust/src/pdsta.rs. Used by tools/pdcap.py and tools/dev/pd_decode_test.py.
"""
from pd_spec import F, get, u16, u32

# KM003C protocol (github.com/okhsunrog/km003c-protocol-research; firmware
# side rust/src/powerz.rs + the capture ring format in rust/src/pd.rs)
KM_VID, KM_PID = 0x5FC9, 0x0063
KM_EP_OUT, KM_EP_IN = 0x01, 0x81
KM_CMD_GET_DATA = 0x0C          # request: cmd, tid, attribute << 1 (LE16)
KM_REQ_ATTRIBUTE = F(1, 15)
KM_PUT_DATA = 0x41              # reply: 4-byte header, 4-byte extended header, payload
KM_HDR_LEN = 8
KM_EXT_HDR = 4
KM_EXT_ATTRIBUTE = F(0, 15)
KM_EXT_SIZE = F(22, 10)
KM_ATT_PD = 0x0010              # PdPacket
KM_REPLY_MAX = 64
# PdPacket payload: measurement preamble, then events
PRE_TS, PRE_VBUS_MV, PRE_IBUS_MA, PRE_CC1_MV, PRE_CC2_MV, PRE_LEN = 0, 4, 6, 8, 10, 12
EV_CONNECTION = 0x45            # 6 B: 0x45, ts LE24, 0x00, code
EV_CONN_TS, EV_CONN_TS_LEN, EV_CONN_CODE, EV_CONN_LEN = 1, 3, 5, 6
EV_WRAPPED = F(7, 1)            # 6 + N B: size flag, ts LE32, sop, N wire bytes
EV_WRAPPED_SIZE = F(0, 6)       # N + 5
EV_WRAPPED_TS, EV_WRAPPED_SOP, EV_WRAPPED_WIRE = 1, 5, 6
EV_WRAPPED_SIZE_BIAS = 5
EV_CONNECT, EV_DISCONNECT = 0x21, 0x22
EV_RX_FULL, EV_RX_RESYNC, EV_BAD_CRC, EV_HARD_RESET = 0x30, 0x31, 0x32, 0x33  # ours (pd.rs CAP_*)
EVENTS = {EV_CONNECT: "*** CONNECT ***", EV_DISCONNECT: "*** DISCONNECT ***",
          EV_RX_FULL: "!!! RX FIFO full: frames may be lost !!!",
          EV_RX_RESYNC: "!!! RX FIFO resync: queued frames flushed !!!",
          EV_BAD_CRC: "!!! bad-CRC packet (not captured) !!!",
          EV_HARD_RESET: "*** HARD RESET ***"}
SOP_NAMES = {0: "", 1: "SOP' ", 2: "SOP'' ", 3: "SOP'_DBG ", 4: "SOP''_DBG "}  # sop byte


def parse_pdpacket(payload, dec):
    """Print the events/messages of one PdPacket payload; returns
    (vbus, ibus, cc1, cc2) from its preamble, or None if too short."""
    if len(payload) < PRE_LEN:
        return None
    ibus = int.from_bytes(payload[PRE_IBUS_MA:PRE_IBUS_MA + 2], "little", signed=True)
    vbus, cc1, cc2 = u16(payload, PRE_VBUS_MV), u16(payload, PRE_CC1_MV), u16(payload, PRE_CC2_MV)
    i = PRE_LEN
    while i < len(payload):
        b0 = payload[i]
        if b0 == EV_CONNECTION:
            if i + EV_CONN_LEN > len(payload):
                break
            code = payload[i + EV_CONN_CODE]
            ets = int.from_bytes(payload[i + EV_CONN_TS:i + EV_CONN_TS + EV_CONN_TS_LEN], "little")
            print("  [%8d ms] %s" % (ets, EVENTS.get(code, "event %02x" % code)))
            i += EV_CONN_LEN
        elif get(b0, EV_WRAPPED):
            wire_len = get(b0, EV_WRAPPED_SIZE) - EV_WRAPPED_SIZE_BIAS
            if wire_len < 0 or i + EV_WRAPPED_WIRE + wire_len > len(payload):
                break
            ets = u32(payload, i + EV_WRAPPED_TS)
            wire = payload[i + EV_WRAPPED_WIRE:i + EV_WRAPPED_WIRE + wire_len]
            sop = payload[i + EV_WRAPPED_SOP]
            line, details = dec.frame(wire, sop)
            print("  [%8d ms] %s%s" % (ets, SOP_NAMES.get(sop, "sop%d " % sop), line))
            for d in details:
                print("               %s" % d)
            i += EV_WRAPPED_WIRE + wire_len
        else:
            break
    return vbus, ibus, cc1, cc2
