"""USB Power Delivery field layouts, as named constants.

Every bit field is an `F(shift, width, unit)`: `width` bits starting at bit
`shift`, one raw step = `unit` (V, A, W, °C, ms, ... as named). `get` reads
the raw value, `val` the scaled one, `pack` builds a value from named fields
(raw or physical) — so a decoder reads `val(pdo, PPS_MAX_V)` and a test
builds `pack((PDO_TYPE, PDO_APDO), (PPS_MAX_V, 21.0), ...)`. Byte-oriented
data blocks (extended messages) get named byte offsets.

Source: USB PD specification Revision 3.2 Version 1.2 (usb.org document
library, "USB Power Delivery"); table numbers below refer to it. Layouts
that differ in Revision 2.0 Version 1.3 (cable / AMA VDOs) are marked
"PD 2.0" with that spec's table numbers.
"""
from typing import NamedTuple


class F(NamedTuple):
    """A bit field: `width` bits at `shift`; one raw step = `unit`."""
    shift: int
    width: int
    unit: float = 1

    @property
    def mask(self):
        return (1 << self.width) - 1


def bit(n):
    """A one-bit flag at bit `n`."""
    return F(n, 1)


def get(v, f):
    """Raw value of field `f` in `v`."""
    return (v >> f.shift) & f.mask


def val(v, f):
    """Scaled value of field `f` in `v` (raw × unit)."""
    return get(v, f) * f.unit


def pack(*fields):
    """Build a value from (field, value) pairs; a value is in the field's
    unit (e.g. volts), converted to raw steps and range-checked."""
    v = 0
    for f, x in fields:
        raw = round(x / f.unit) if f.unit != 1 else int(x)
        assert 0 <= raw <= f.mask, "%r out of range for %r" % (x, f)
        v |= raw << f.shift
    return v


def u16(d, off):
    """Little-endian u16 at byte `off` (None when the block is too short)."""
    return d[off] | d[off + 1] << 8 if off + 2 <= len(d) else None


def u32(d, off):
    return int.from_bytes(d[off:off + 4], "little") if off + 4 <= len(d) else None


# ---- 6.2.1 Message Header (Table 6.2) ----
HDR_EXTENDED = bit(15)
HDR_NUM_OBJECTS = F(12, 3)
HDR_MSG_ID = F(9, 3)
HDR_POWER_ROLE = bit(8)    # SOP: 1 = source
HDR_CABLE_PLUG = bit(8)    # SOP'/SOP'': 1 = sent by a cable plug / VPD
HDR_SPEC_REV = F(6, 2)
HDR_DATA_ROLE = bit(5)     # SOP: 1 = DFP
HDR_MSG_TYPE = F(0, 5)
REV_1_0, REV_2_0, REV_3_X = 0, 1, 2
REV_NAMES = {REV_1_0: "1.0", REV_2_0: "2.0", REV_3_X: "3.x"}

# ---- message types (Tables 6.4 / 6.5 / 6.47) ----
CTRL_GOODCRC, CTRL_GOTOMIN, CTRL_ACCEPT, CTRL_REJECT, CTRL_PING, CTRL_PS_RDY = 1, 2, 3, 4, 5, 6
CTRL_GET_SOURCE_CAP, CTRL_GET_SINK_CAP, CTRL_DR_SWAP, CTRL_PR_SWAP = 7, 8, 9, 10
CTRL_VCONN_SWAP, CTRL_WAIT, CTRL_SOFT_RESET, CTRL_DATA_RESET = 11, 12, 13, 14
CTRL_DATA_RESET_COMPLETE, CTRL_NOT_SUPPORTED, CTRL_GET_SOURCE_CAP_EXT = 15, 16, 17
CTRL_GET_STATUS, CTRL_FR_SWAP, CTRL_GET_PPS_STATUS, CTRL_GET_COUNTRY_CODES = 18, 19, 20, 21
CTRL_GET_SINK_CAP_EXT, CTRL_GET_SOURCE_INFO, CTRL_GET_REVISION = 22, 23, 24
CTRL_MSGS = {CTRL_GOODCRC: "GoodCRC", CTRL_GOTOMIN: "GotoMin", CTRL_ACCEPT: "Accept",
             CTRL_REJECT: "Reject", CTRL_PING: "Ping", CTRL_PS_RDY: "PS_RDY",
             CTRL_GET_SOURCE_CAP: "Get_Source_Cap", CTRL_GET_SINK_CAP: "Get_Sink_Cap",
             CTRL_DR_SWAP: "DR_Swap", CTRL_PR_SWAP: "PR_Swap", CTRL_VCONN_SWAP: "VCONN_Swap",
             CTRL_WAIT: "Wait", CTRL_SOFT_RESET: "Soft_Reset", CTRL_DATA_RESET: "Data_Reset",
             CTRL_DATA_RESET_COMPLETE: "Data_Reset_Complete", CTRL_NOT_SUPPORTED: "Not_Supported",
             CTRL_GET_SOURCE_CAP_EXT: "Get_Source_Cap_Ext", CTRL_GET_STATUS: "Get_Status",
             CTRL_FR_SWAP: "FR_Swap", CTRL_GET_PPS_STATUS: "Get_PPS_Status",
             CTRL_GET_COUNTRY_CODES: "Get_Country_Codes", CTRL_GET_SINK_CAP_EXT: "Get_Sink_Cap_Ext",
             CTRL_GET_SOURCE_INFO: "Get_Source_Info", CTRL_GET_REVISION: "Get_Revision"}

DATA_SOURCE_CAP, DATA_REQUEST, DATA_BIST, DATA_SINK_CAP, DATA_BATTERY_STATUS = 1, 2, 3, 4, 5
DATA_ALERT, DATA_GET_COUNTRY_INFO, DATA_ENTER_USB, DATA_EPR_REQUEST = 6, 7, 8, 9
DATA_EPR_MODE, DATA_SOURCE_INFO, DATA_REVISION, DATA_VENDOR_DEFINED = 10, 11, 12, 15
DATA_MSGS = {DATA_SOURCE_CAP: "Source_Cap", DATA_REQUEST: "Request", DATA_BIST: "BIST",
             DATA_SINK_CAP: "Sink_Cap", DATA_BATTERY_STATUS: "Battery_Status", DATA_ALERT: "Alert",
             DATA_GET_COUNTRY_INFO: "Get_Country_Info", DATA_ENTER_USB: "Enter_USB",
             DATA_EPR_REQUEST: "EPR_Request", DATA_EPR_MODE: "EPR_Mode",
             DATA_SOURCE_INFO: "Source_Info", DATA_REVISION: "Revision",
             DATA_VENDOR_DEFINED: "Vendor"}

EXT_SOURCE_CAP_EXT, EXT_STATUS, EXT_GET_BATTERY_CAP, EXT_GET_BATTERY_STATUS = 1, 2, 3, 4
EXT_BATTERY_CAP, EXT_GET_MANUFACTURER_INFO, EXT_MANUFACTURER_INFO = 5, 6, 7
EXT_SECURITY_REQUEST, EXT_SECURITY_RESPONSE, EXT_FW_UPDATE_REQUEST = 8, 9, 10
EXT_FW_UPDATE_RESPONSE, EXT_PPS_STATUS, EXT_COUNTRY_INFO, EXT_COUNTRY_CODES = 11, 12, 13, 14
EXT_SINK_CAP_EXT, EXT_EXTENDED_CONTROL, EXT_EPR_SOURCE_CAP, EXT_EPR_SINK_CAP = 15, 16, 17, 18
EXT_VENDOR_DEFINED = 30
EXT_MSGS = {EXT_SOURCE_CAP_EXT: "Source_Cap_Ext", EXT_STATUS: "Status",
            EXT_GET_BATTERY_CAP: "Get_Battery_Cap", EXT_GET_BATTERY_STATUS: "Get_Battery_Status",
            EXT_BATTERY_CAP: "Battery_Cap", EXT_GET_MANUFACTURER_INFO: "Get_Manufacturer_Info",
            EXT_MANUFACTURER_INFO: "Manufacturer_Info", EXT_SECURITY_REQUEST: "Security_Request",
            EXT_SECURITY_RESPONSE: "Security_Response", EXT_FW_UPDATE_REQUEST: "FW_Update_Request",
            EXT_FW_UPDATE_RESPONSE: "FW_Update_Response", EXT_PPS_STATUS: "PPS_Status",
            EXT_COUNTRY_INFO: "Country_Info", EXT_COUNTRY_CODES: "Country_Codes",
            EXT_SINK_CAP_EXT: "Sink_Cap_Ext", EXT_EXTENDED_CONTROL: "Extended_Control",
            EXT_EPR_SOURCE_CAP: "EPR_Source_Cap", EXT_EPR_SINK_CAP: "EPR_Sink_Cap",
            EXT_VENDOR_DEFINED: "Vendor_Ext"}

# ---- 6.5.1 Extended Message Header (Table 6.48) ----
EXTHDR_CHUNKED = bit(15)
EXTHDR_CHUNK_NUMBER = F(11, 4)
EXTHDR_REQUEST_CHUNK = bit(10)
EXTHDR_DATA_SIZE = F(0, 9)
MAX_EXT_CHUNK_LEN = 26  # MaxExtendedMsgChunkLen (Table 6.65)

# ---- 6.4.1.3 Power Data Objects (Tables 6.6 - 6.18) ----
PDO_TYPE = F(30, 2)
PDO_FIXED, PDO_BATTERY, PDO_VARIABLE, PDO_APDO = 0, 1, 2, 3
APDO_TYPE = F(28, 2)
APDO_PPS, APDO_EPR_AVS, APDO_SPR_AVS = 0, 1, 2

FIXED_VOLTAGE = F(10, 10, 0.05)
FIXED_MAX_CURRENT = F(0, 10, 0.01)
FIXED_PEAK_CURRENT = F(20, 2)  # source only (Table 6.18)
PEAK_NAMES = {1: "peak OL1 (150/125/110 %)", 2: "peak OL2 (200/150/125 %)",
              3: "peak OL3 (200/175/150 %)"}
# the 5 V fixed PDO in position 1 carries the port's flags (Tables 6.8 / 6.9)
SRC_DUAL_ROLE_POWER = bit(29)
SRC_USB_SUSPEND = bit(28)
SRC_UNCONSTRAINED = bit(27)
SRC_USB_COMMS = bit(26)
SRC_DUAL_ROLE_DATA = bit(25)
SRC_UNCHUNKED_EXT = bit(24)
SRC_EPR_CAPABLE = bit(23)
SNK_DUAL_ROLE_POWER = bit(29)
SNK_HIGHER_CAPABILITY = bit(28)
SNK_UNCONSTRAINED = bit(27)
SNK_USB_COMMS = bit(26)
SNK_DUAL_ROLE_DATA = bit(25)
SNK_FRS_CURRENT = F(23, 2)
FRS_NAMES = {1: "default", 2: "1.5 A", 3: "3.0 A"}

BATTERY_MAX_VOLTAGE = F(20, 10, 0.05)
BATTERY_MIN_VOLTAGE = F(10, 10, 0.05)
BATTERY_MAX_POWER = F(0, 10, 0.25)
VARIABLE_MAX_VOLTAGE = F(20, 10, 0.05)
VARIABLE_MIN_VOLTAGE = F(10, 10, 0.05)
VARIABLE_MAX_CURRENT = F(0, 10, 0.01)

PPS_POWER_LIMITED = bit(27)  # source only
PPS_MAX_VOLTAGE = F(17, 8, 0.1)
PPS_MIN_VOLTAGE = F(8, 8, 0.1)
PPS_MAX_CURRENT = F(0, 7, 0.05)
SPR_AVS_PEAK_CURRENT = F(26, 2)
SPR_AVS_MAX_CURRENT_15V = F(10, 10, 0.01)  # 9-15 V
SPR_AVS_MAX_CURRENT_20V = F(0, 10, 0.01)   # 15-20 V
EPR_AVS_PEAK_CURRENT = F(26, 2)
EPR_AVS_MAX_VOLTAGE = F(17, 9, 0.1)
EPR_AVS_MIN_VOLTAGE = F(8, 8, 0.1)
EPR_AVS_PDP = F(0, 8)  # W

# ---- 6.4.2 Request Data Objects (Tables 6.19 - 6.22) ----
RDO_OBJECT_POSITION = F(28, 4)
RDO_CAPABILITY_MISMATCH = bit(26)
RDO_USB_COMMS = bit(25)
RDO_NO_USB_SUSPEND = bit(24)
RDO_UNCHUNKED_EXT = bit(23)
RDO_EPR_CAPABLE = bit(22)
RDO_OPERATING_CURRENT = F(10, 10, 0.01)  # fixed / variable
RDO_MAX_CURRENT = F(0, 10, 0.01)
RDO_OPERATING_POWER = F(10, 10, 0.25)    # battery
RDO_MAX_POWER = F(0, 10, 0.25)
RDO_PPS_VOLTAGE = F(9, 12, 0.02)
RDO_AVS_VOLTAGE = F(9, 12, 0.025)
RDO_APDO_CURRENT = F(0, 7, 0.05)         # PPS / AVS
EPR_SPR_SLOTS = 7  # EPR caps: positions 1-7 SPR (zero-filled), 8+ EPR (6.5.18)

# ---- 6.4.3 BIST (Table 6.23) ----
BIST_MODE = F(28, 4)
BIST_MODES = {5: "carrier", 8: "test data", 9: "shared test mode entry", 10: "shared test mode exit"}

# ---- 6.4.4 Battery Status Data Object (Table 6.24) ----
BSDO_CAPACITY = F(16, 16, 0.1)  # Wh; raw 0xFFFF = unknown
BSDO_CAPACITY_UNKNOWN = 0xFFFF
BSDO_CHARGING_STATUS = F(10, 2)
BSDO_CHARGING_NAMES = {0: "charging", 1: "discharging", 2: "idle"}
BSDO_PRESENT = bit(9)
BSDO_INVALID_REF = bit(8)

# ---- 6.4.5 Alert Data Object (Table 6.25) ----
ADO_EXTENDED = bit(31)
ADO_EVENTS = {bit(30): "OVP", bit(29): "source input change", bit(28): "operating condition change",
              bit(27): "OTP", bit(26): "OCP", bit(25): "battery status change"}
ADO_BATTERY_STATUS_CHANGE = bit(25)
ADO_FIXED_BATTERIES = F(20, 4)
ADO_HOT_SWAP_BATTERIES = F(16, 4)
ADO_EXTENDED_EVENT = F(0, 4)
ADO_EXTENDED_EVENTS = {1: "power state change", 2: "power button press", 3: "power button release",
                       4: "controller initiated wake", 5: "source reducing capabilities"}

# ---- 6.4.6 Country Code Data Object (Table 6.26) ----
CCDO_CHAR1 = F(24, 8)
CCDO_CHAR2 = F(16, 8)

# ---- 6.4.7 Enter_USB Data Object (Table 6.27) ----
EUDO_USB_MODE = F(28, 3)
EUDO_USB_MODES = {0: "USB2.0", 1: "USB3.2", 2: "USB4"}
EUDO_FLAGS = {bit(26): "USB4 DRD", bit(25): "USB3 DRD", bit(16): "PCIe", bit(15): "DP", bit(14): "TBT",
              bit(13): "host present"}
EUDO_CABLE_SPEED = F(21, 3)
EUDO_CABLE_TYPE = F(19, 2)
EUDO_CABLE_TYPES = {0: "passive", 1: "active re-timer", 2: "active re-driver", 3: "optically isolated"}
EUDO_CABLE_CURRENT = F(17, 2)
EUDO_CABLE_CURRENTS = {0: "no VBUS", 1: "no VBUS", 2: "3 A", 3: "5 A"}

# ---- 6.4.9 EPR Mode Data Object (Table 6.28) ----
EPRMDO_ACTION = F(24, 8)
EPRMDO_DATA = F(16, 8)
EPR_ENTER, EPR_ENTER_ACK, EPR_ENTER_SUCCEEDED, EPR_ENTER_FAILED, EPR_EXIT = 1, 2, 3, 4, 5
EPRMDO_ACTIONS = {EPR_ENTER: "enter EPR mode", EPR_ENTER_ACK: "enter acknowledged",
                  EPR_ENTER_SUCCEEDED: "enter succeeded", EPR_ENTER_FAILED: "enter failed",
                  EPR_EXIT: "exit EPR mode"}
EPR_FAIL_REASONS = {0: "unknown", 1: "cable not EPR capable", 2: "source failed to become VCONN source",
                    3: "EPR capable not set in RDO", 4: "source unable to enter EPR now",
                    5: "EPR capable not set in PDO"}

# ---- 6.4.10 Source_Info Data Objects (Tables 6.29 / 6.30) ----
SIDO_PORT_TYPE = bit(31)  # 1 = guaranteed capability
SIDO1_MAX_PDP = F(16, 8)
SIDO1_PRESENT_PDP = F(8, 8)
SIDO1_REPORTED_PDP = F(0, 8)
SIDO2_DPS_PORT = bit(30)
SIDO2_MAX_PDP = F(9, 9, 0.5)
SIDO2_GUARANTEED_PDP = F(0, 9, 0.5)

# ---- 6.4.11 Revision Message Data Object (Table 6.31) ----
RMDO_REV_MAJOR = F(28, 4)
RMDO_REV_MINOR = F(24, 4)
RMDO_VER_MAJOR = F(20, 4)
RMDO_VER_MINOR = F(16, 4)

# ---- 6.4.12 VDM header (Tables 6.32 / 6.33) ----
VDM_SVID = F(16, 16)
VDM_STRUCTURED = bit(15)
VDM_VERSION_MAJOR = F(13, 2)  # 0 = SVDM 1.0, 1 = 2.x
VDM_VERSION_MINOR = F(11, 2)
VDM_OBJECT_POSITION = F(8, 3)
VDM_EXIT_ALL_MODES = 7
VDM_COMMAND_TYPE = F(6, 2)
VDM_REQ, VDM_ACK, VDM_NAK, VDM_BUSY = 0, 1, 2, 3
VDM_TYPES = {VDM_REQ: "REQ", VDM_ACK: "ACK", VDM_NAK: "NAK", VDM_BUSY: "BUSY"}
VDM_COMMAND = F(0, 5)
CMD_DISCOVER_IDENTITY, CMD_DISCOVER_SVIDS, CMD_DISCOVER_MODES = 1, 2, 3
CMD_ENTER_MODE, CMD_EXIT_MODE, CMD_ATTENTION = 4, 5, 6
CMD_SVID_SPECIFIC = 16  # 16-31
VDM_CMDS = {CMD_DISCOVER_IDENTITY: "Discover Identity", CMD_DISCOVER_SVIDS: "Discover SVIDs",
            CMD_DISCOVER_MODES: "Discover Modes", CMD_ENTER_MODE: "Enter Mode",
            CMD_EXIT_MODE: "Exit Mode", CMD_ATTENTION: "Attention"}
SVID_PD, SVID_DISPLAYPORT = 0xFF00, 0xFF01
SVID_NAMES = {SVID_PD: "PD SID", SVID_DISPLAYPORT: "DisplayPort"}
SVIDS_PER_VDO = (F(16, 16), F(0, 16))  # Discover SVIDs responder VDO (Table 6.46)

# ---- 6.4.12.3.1 ID Header VDO (Table 6.34) ----
IDH_USB_HOST = bit(31)
IDH_USB_DEVICE = bit(30)
IDH_UFP_TYPE = F(27, 3)   # SOP: product type (UFP); SOP': cable plug / VPD
IDH_MODAL = bit(26)
IDH_DFP_TYPE = F(23, 3)   # PD 3.x, SOP only
IDH_CONNECTOR = F(21, 2)  # PD 3.x
IDH_VID = F(0, 16)
UFP_NOT_UFP, UFP_HUB, UFP_PERIPHERAL, UFP_PSD, UFP_AMA = 0, 1, 2, 3, 5
CABLE_NOT_CABLE, CABLE_PASSIVE, CABLE_ACTIVE, CABLE_VPD = 0, 3, 4, 6
DFP_NOT_DFP, DFP_HUB, DFP_HOST, DFP_POWER_BRICK, DFP_AMC = 0, 1, 2, 3, 4
CONNECTOR_RECEPTACLE, CONNECTOR_PLUG = 2, 3
UFP_TYPE_NAMES = {UFP_HUB: "hub", UFP_PERIPHERAL: "peripheral", UFP_PSD: "PSD", UFP_AMA: "AMA"}
CABLE_TYPE_NAMES = {CABLE_NOT_CABLE: "not a cable", CABLE_PASSIVE: "passive cable",
                    CABLE_ACTIVE: "active cable", CABLE_VPD: "VPD"}
DFP_TYPE_NAMES = {DFP_HUB: "hub", DFP_HOST: "host", DFP_POWER_BRICK: "power brick", DFP_AMC: "AMC"}
CONNECTOR_NAMES = {CONNECTOR_RECEPTACLE: "receptacle", CONNECTOR_PLUG: "plug"}

# ---- Cert Stat / Product VDOs (Tables 6.38 / 6.39) ----
PRODUCT_PID = F(16, 16)
PRODUCT_BCD = F(0, 16)

# ---- USB speed fields (UFP VDO, cable VDOs, Enter_USB) ----
USB_SPEEDS = {0: "USB 2.0 only", 1: "USB3 Gen1", 2: "USB3/USB4 Gen2", 3: "USB4 Gen3", 4: "USB4 Gen4"}
USB_SPEEDS_PD20 = {0: "USB 2.0 only", 1: "USB 3.1 Gen1", 2: "USB 3.1 Gen1 and Gen2"}

# ---- cable VDOs: passive (Table 6.42), active VDO1 (Table 6.43); PD 2.0 6-28 / 6-29 ----
CABLE_HW_VERSION = F(28, 4)
CABLE_FW_VERSION = F(24, 4)
CABLE_VDO_VERSION = F(21, 3)  # PD 3.x
CABLE_PLUG_TYPE = F(18, 2)
CABLE_PLUG_NAMES = {0: "Type-A", 1: "Type-B", 2: "Type-C", 3: "captive"}
CABLE_EPR_CAPABLE = bit(17)   # PD 3.x
CABLE_LATENCY = F(13, 4)      # 1-7 ~n m; passive 8 >7 m; active 8/9/10 ~100/200/300 m
CABLE_TERMINATION = F(11, 2)  # passive: 1 = VCONN required; active: 2 one end, 3 both ends
CABLE_MAX_VBUS = F(9, 2)      # PD 3.x: 0 20 V, 3 50 V (1/2 deprecated 30/40 V)
CABLE_MAX_VBUS_NAMES = {0: "20 V", 3: "50 V"}
ACTIVE_SBU_NOT_SUPPORTED = bit(8)  # PD 3.x active
ACTIVE_SBU_ACTIVE = bit(7)
CABLE_VBUS_CURRENT = F(5, 2)
CABLE_CURRENT_NAMES = {1: "3 A", 2: "5 A"}
CABLE_VBUS_THROUGH = bit(4)        # active; PD 2.0 passive too
ACTIVE_SOP2_CONTROLLER = bit(3)    # active
CABLE_USB_SPEED = F(0, 3)

# ---- Active Cable VDO2 (Table 6.44) ----
ACTIVE2_MAX_TEMP = F(24, 8)       # °C
ACTIVE2_SHUTDOWN_TEMP = F(16, 8)  # °C
ACTIVE2_OPTICAL = bit(10)
ACTIVE2_RETIMER = bit(9)
ACTIVE2_NO_USB4 = bit(8)
ACTIVE2_NO_USB2 = bit(5)
ACTIVE2_NO_USB3 = bit(4)
ACTIVE2_TWO_LANES = bit(3)
ACTIVE2_OPTICALLY_ISOLATED = bit(2)
ACTIVE2_USB4_ASYMMETRIC = bit(1)
ACTIVE2_GEN2 = bit(0)

# ---- VCONN Powered USB Device VDO (Table 6.45) ----
VPD_HW_VERSION = F(28, 4)
VPD_FW_VERSION = F(24, 4)
VPD_CT_CURRENT_5A = bit(14)
VPD_VBUS_IMPEDANCE = F(7, 6)  # 2 mOhm steps
VPD_GND_IMPEDANCE = F(1, 6)   # 1 mOhm steps
VPD_CHARGE_THROUGH = bit(0)

# ---- UFP VDO (Table 6.40) ----
UFP_VDO_VERSION = F(29, 3)  # 3 = v1.3
UFP_USB4_DEVICE = bit(27)
UFP_USB32_DEVICE = bit(26)
UFP_USB2_DEVICE = F(24, 2)
UFP_USB2_NAMES = {1: "USB2 billboard only", 2: "USB2 device"}
UFP_VCONN_POWER = F(8, 3)
VCONN_POWER_NAMES = ["1", "1.5", "2", "3", "4", "5", "6", "?"]  # W
UFP_VCONN_REQUIRED = bit(7)
UFP_ALT_MODES = {bit(5): "alt modes without reconfig", bit(4): "non-TBT3 reconfig alt modes",
                 bit(3): "TBT3 alt mode"}
UFP_USB_SPEED = F(0, 3)

# ---- DFP VDO (Table 6.41) ----
DFP_VDO_VERSION = F(29, 3)  # 2 = v1.2
DFP_HOSTS = {bit(26): "USB4 host", bit(25): "USB3.2 host", bit(24): "USB2 host"}
DFP_PORT_NUMBER = F(0, 5)

# ---- PD 2.0 Alternate Mode Adapter VDO (PD 2.0 Table 6-30) ----
AMA_HW_VERSION = F(28, 4)
AMA_FW_VERSION = F(24, 4)
AMA_VCONN_POWER = F(5, 3)
AMA_VCONN_REQUIRED = bit(4)
AMA_VBUS_REQUIRED = bit(3)
AMA_USB_SPEED = F(0, 3)
AMA_USB_SPEEDS = {0: "USB 2.0 only", 1: "USB 3.1 Gen1", 2: "USB 3.1 Gen1/Gen2", 3: "billboard only"}

# ---- extended data blocks: byte offsets ----
# Source Capabilities Extended Data Block (Table 6.50)
SCEDB_VID, SCEDB_PID, SCEDB_XID, SCEDB_FW_VERSION, SCEDB_HW_VERSION = 0, 2, 4, 8, 9
SCEDB_VOLTAGE_REGULATION, SCEDB_HOLDUP_TIME, SCEDB_COMPLIANCE, SCEDB_TOUCH_CURRENT = 10, 11, 12, 13
SCEDB_PEAK_CURRENTS = (14, 16, 18)
SCEDB_TOUCH_TEMP, SCEDB_SOURCE_INPUTS, SCEDB_BATTERIES, SCEDB_SPR_PDP, SCEDB_EPR_PDP = 20, 21, 22, 23, 24
SCEDB_LEN = 25
LOAD_STEP = F(0, 2)  # 1 = 500 mA/us, else 150 mA/us (also SKEDB Load Step)
VOLTAGE_REGULATION_IOC_90 = bit(2)
COMPLIANCE_BITS = {bit(0): "LPS", bit(1): "PS1", bit(2): "PS2"}
TOUCH_CURRENT_BITS = {bit(0): "low touch current EPS", bit(1): "ground pin",
                      bit(2): "ground pin is protective earth"}
SCEDB_TOUCH_TEMPS = {0: "IEC 60950-1", 1: "IEC 62368-1 TS1", 2: "IEC 62368-1 TS2"}
INPUT_EXTERNAL_SUPPLY = bit(0)
INPUT_EXTERNAL_UNCONSTRAINED = bit(1)
INPUT_INTERNAL_BATTERY = bit(2)
BATTERIES_FIXED = F(0, 4)
BATTERIES_HOT_SWAP = F(4, 4)
# Peak Current 1-3 (Table 6.50) = Sink Load Characteristics (Table 6.61)
PEAK_OVERLOAD = F(0, 5, 10)    # %; raw > 25 is treated as 25
PEAK_OVERLOAD_MAX = 25
PEAK_PERIOD = F(5, 6, 20)      # ms
PEAK_DUTY_CYCLE = F(11, 4, 5)  # %
PEAK_VBUS_DROOP = bit(15)

# Sink Capabilities Extended Data Block (Table 6.61)
SKEDB_VID, SKEDB_PID, SKEDB_XID, SKEDB_FW_VERSION, SKEDB_HW_VERSION = 0, 2, 4, 8, 9
SKEDB_VERSION, SKEDB_LOAD_STEP, SKEDB_LOAD_CHARACTERISTICS, SKEDB_COMPLIANCE = 10, 11, 12, 14
SKEDB_TOUCH_TEMP, SKEDB_BATTERY_INFO, SKEDB_SINK_MODES = 15, 16, 17
SKEDB_SPR_MIN_PDP, SKEDB_SPR_OPERATIONAL_PDP, SKEDB_SPR_MAX_PDP = 18, 19, 20
SKEDB_EPR_MIN_PDP, SKEDB_EPR_OPERATIONAL_PDP, SKEDB_EPR_MAX_PDP = 21, 22, 23
SKEDB_LEN = 24
SKEDB_TOUCH_TEMPS = {0: "none", 1: "IEC 60950-1", 2: "IEC 62368-1 TS1", 3: "IEC 62368-1 TS2"}
SINK_MODE_BITS = {bit(0): "PPS charging", bit(1): "VBUS powered", bit(2): "AC powered",
                  bit(3): "battery powered", bit(4): "battery essentially unlimited", bit(5): "AVS"}

# SOP Status Data Block (Table 6.51)
SDB_INTERNAL_TEMP, SDB_PRESENT_INPUT, SDB_PRESENT_BATTERY_INPUT, SDB_EVENT_FLAGS = 0, 1, 2, 3
SDB_TEMPERATURE_STATUS, SDB_POWER_STATUS, SDB_POWER_STATE_CHANGE = 4, 5, 6
TEMP_NOT_SUPPORTED, TEMP_BELOW_2C = 0, 1  # internal temperature, else °C
INPUT_EXTERNAL_POWER = F(1, 2)
INPUT_EXTERNAL_NAMES = {0: "internally powered", 1: "DC external power", 3: "AC external power"}
INPUT_FROM_BATTERY = bit(3)
INPUT_FROM_NON_BATTERY = bit(4)
EVENT_BITS = {bit(1): "OCP", bit(2): "OTP", bit(3): "OVP"}
EVENT_PPS_CL_MODE = bit(4)
TEMPERATURE_STATUS = F(1, 2)
TEMPERATURE_STATUS_NAMES = {0: "not supported", 1: "normal", 2: "warning", 3: "over-temperature"}
POWER_LIMIT_BITS = {bit(1): "cable current", bit(2): "other ports", bit(3): "external power",
                    bit(4): "event flags", bit(5): "temperature"}
POWER_STATE = F(0, 3)
POWER_STATE_NAMES = {1: "S0", 2: "modern standby", 3: "S3", 4: "S4", 5: "S5", 6: "G3"}
POWER_INDICATOR = F(3, 3)
POWER_INDICATOR_NAMES = {0: "off", 1: "on", 2: "blinking", 3: "breathing"}
# SOP'/SOP'' Status (Table 6.52)
CABLE_SDB_TEMP, CABLE_SDB_FLAGS = 0, 1
CABLE_THERMAL_SHUTDOWN = bit(0)

# Get Battery Cap / Status Data Blocks (Tables 6.53 / 6.54): byte 0 = battery ref
BATTERY_REF = 0
FIXED_BATTERY_SLOTS = 4  # refs 0-3 fixed, 4-7 hot-swappable
# Battery Capability Data Block (Table 6.55)
BCDB_VID, BCDB_PID, BCDB_DESIGN_CAPACITY, BCDB_LAST_FULL_CAPACITY, BCDB_BATTERY_TYPE = 0, 2, 4, 6, 8
BCDB_LEN = 9
BATTERY_CAPACITY_UNIT = 0.1  # Wh
BATTERY_NOT_PRESENT, BATTERY_CAPACITY_UNKNOWN = 0, 0xFFFF
BCDB_INVALID_REF = bit(0)
# Get_Manufacturer_Info (Table 6.56) / Manufacturer_Info (Table 6.57)
GMIDB_TARGET, GMIDB_REF = 0, 1
MANUFACTURER_TARGET_PORT, MANUFACTURER_TARGET_BATTERY = 0, 1
MIDB_VID, MIDB_PID, MIDB_STRING = 0, 2, 4
# PPS Status Data Block (Table 6.58)
PPSSDB_OUTPUT_VOLTAGE, PPSSDB_OUTPUT_CURRENT, PPSSDB_REAL_TIME_FLAGS = 0, 2, 3
PPSSDB_LEN = 4
PPS_VOLTAGE_UNIT, PPS_VOLTAGE_NOT_SUPPORTED = 0.02, 0xFFFF
PPS_CURRENT_UNIT, PPS_CURRENT_NOT_SUPPORTED = 0.05, 0xFF
PPS_TEMPERATURE_FLAG = F(1, 2)  # names as TEMPERATURE_STATUS_NAMES
PPS_CL_MODE = bit(3)
# Country Info / Country Codes Data Blocks (Tables 6.59 / 6.60)
CIDB_COUNTRY_CODE, CIDB_DATA = 0, 4
CCDB_LENGTH, CCDB_CODES = 0, 2
# Extended Control Data Block (Tables 6.62 / 6.63)
ECDB_TYPE, ECDB_DATA = 0, 1
ECDB_TYPES = {1: "EPR_Get_Source_Cap", 2: "EPR_Get_Sink_Cap", 3: "EPR_KeepAlive", 4: "EPR_KeepAlive_Ack"}
# Vendor_Defined_Extended (Table 6.64)
VDEM_SVID, VDEM_COMMAND, VDEM_DATA = 0, 2, 4
VDEM_COMMAND_SPACE = F(0, 15)
