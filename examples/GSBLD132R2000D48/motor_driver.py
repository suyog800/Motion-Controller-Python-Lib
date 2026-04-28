
"""
motor_driver.py — EDS indices sourced strictly from MotionController_UserAPI.csv
This file uses ONLY the indices defined in your CSV (no dynamic EDS lookups).

Patches based on your Qt backend patterns:
- connect(): reads TPDO/RPDO mapping from device (tpdo.read()/rpdo.read()).
- _sdo_write(): supports VAR (no subindex) and arrays/records.
- Float-friendly API inputs (no forced int(...) unless you change types).
- _pdo_write_by_index(): refreshes PDO cache and retries once before failing.
- _try_write_same_pdo(): matches by (index, subindex), writes via .phys if possible.
"""

from __future__ import annotations

import logging
from typing import Optional, List, Tuple
import canopen
from canopen.objectdictionary import REAL32, REAL64
log = logging.getLogger(__name__)
import can

# ----------------------- Auto-generated from MotionController_UserAPI.csv -----------------------
CONTROLWORD_INDEX = 0x6040
CONTROLWORD_SUB   = 0
MODES_OF_OPERATION_INDEX = 0x6060
MODES_OF_OPERATION_SUB   = 0
IQ_KP_INDEX = 0x2200
IQ_KP_SUB   = 0
IQ_KI_INDEX = 0x2201
IQ_KI_SUB   = 0
SPEED_KP_INDEX = 0x2202
SPEED_KP_SUB   = 0
SPEED_KI_INDEX = 0x2203
SPEED_KI_SUB   = 0
SPEED_KD_INDEX = 0x2204
SPEED_KD_SUB   = 0
SPEED_TC_INDEX = 0x2205
SPEED_TC_SUB   = 0
POS_KP_INDEX = 0x2206
POS_KP_SUB   = 0
POS_KI_INDEX = 0x2207
POS_KI_SUB   = 0
POS_KD_INDEX = 0x2208
POS_KD_SUB   = 0
POS_TC_INDEX = 0x2209
POS_TC_SUB   = 0
TARGET_IQ_INDEX = 0x2030
TARGET_IQ_SUB   = 0
TARGET_POSITION_INDEX = 0x607A
TARGET_POSITION_SUB   = 0
TARGET_VELOCITY_INDEX = 0x60FF
TARGET_VELOCITY_SUB   = 0
TARGET_ACCELERATION_INDEX = 0x6083
TARGET_ACCELERATION_SUB   = 0
TARGET_DECELERATION_INDEX = 0x6084
TARGET_DECELERATION_SUB   = 0
HEARTBEAT_INDEX = 0x1017
HEARTBEAT_SUB   = 0
# -----------------------------------------------------------------------------------------------

class _TxLogger(can.Listener):
    def on_message_sent(self, msg: can.Message) -> None:
        # arbitration_id is the "CAN address" (COB-ID for CANopen)
        data_hex = msg.data.hex(" ").upper()
        print(f"TX 0x{msg.arbitration_id:03X}  [{msg.dlc}]  {data_hex}")

class Motor_Driver:
    def __init__(self, node_id: int, eds_path: str = "DS301_profile.eds",
                 bustype: str = "socketcan", 
                 channel: str = "can0", 
                 bitrate: Optional[int] = None,
                 motor_config_path: Optional[str] = None):
        self.node_id = int(node_id)
        self.eds_path = eds_path
        self.bustype = bustype
        self.channel = channel
        self.bitrate = bitrate
        self.network: Optional[canopen.Network] = None
        self.node: Optional[canopen.RemoteNode] = None

        self.controlword_val = 0
        self.ModeofOperation_val = 1
        self.targetiq_val = 0

        self.iq_kp_val = 0.45
        self.iq_ki_val = 116.82

        self.speedkp_val = 30
        self.speedki_cal = 4218
        self.speedkd_val = 0
        self.speedTC_val = 0

        self.positionkp_val = 1250
        self.positionki_val = 0
        self.positionkd_val = 0
        self.positionTc_val = 0

        # Motor info
        self.model_name = None
        self.pole_pairs = None
        self.position_sensor = {}
        self.electrical = {}

        if motor_config_path is not None:
            self._load_motor_config(motor_config_path)

                # Runtime feedback (updated via TPDO callbacks)
        self.position_measured = 0.0   # 0x6064, REAL64
        self.velocity_measured = 0.0   # 0x606C, REAL32
        self.iq_measured = 0.0         # 0x2032, REAL32
        self.id_measured = 0.0         # 0x2033, REAL32

        # Indicates if new TPDO data has arrived since last read
        self.new_state_available = False



    def _load_motor_config(self, path: str) -> None:
        cfg_path = Path(path)
        with cfg_path.open("r") as f:
            cfg = json.load(f)

        self.model_name = cfg.get("model_name")
        self.pole_pairs = cfg.get("pole_pairs")

        self.position_sensor = cfg.get("position_sensor", {})
        self.electrical = cfg.get("electrical", {})

        gains = cfg.get("default_gains", {})

        # Iq gains
        iq_cfg = gains.get("iq", {})
        self.iq_kp_val = iq_cfg.get("kp", self.iq_kp_val)
        self.iq_ki_val = iq_cfg.get("ki", self.iq_ki_val)

        # Speed gains
        sp_cfg = gains.get("speed", {})
        self.speedkp_val = sp_cfg.get("kp", self.speedkp_val)
        self.speedki_cal = sp_cfg.get("ki", self.speedki_cal)
        self.speedkd_val = sp_cfg.get("kd", self.speedkd_val)
        self.speedTC_val = sp_cfg.get("tc", self.speedTC_val)

        # Position gains
        pos_cfg = gains.get("position", {})
        self.positionkp_val = pos_cfg.get("kp", self.positionkp_val)
        self.positionki_val = pos_cfg.get("ki", self.positionki_val)
        self.positionkd_val = pos_cfg.get("kd", self.positionkd_val)
        self.positionTc_val = pos_cfg.get("tc", self.positionTc_val)

    # ---- Lifecycle ----
    def connect(self) -> None:
        self.network = canopen.Network()
        self.network.connect(bustype=self.bustype, channel=self.channel, bitrate=self.bitrate)
        # # --- TX hook: print every outgoing CAN frame (COB-ID + payload) ---
        # try:
        #     bus = self.network.bus  # python-can Bus instance
        #     self._orig_can_send = bus.send

        #     def _send_with_log(msg, *args, **kwargs):
        #         data_hex = msg.data.hex(" ").upper()
        #         print(f"TX 0x{msg.arbitration_id:03X}  [{msg.dlc}]  {data_hex}")
        #         return self._orig_can_send(msg, *args, **kwargs)

        #     bus.send = _send_with_log
        # except Exception as e:
        #     log.debug("Failed to hook CAN TX send(): %s", e)

        self.node = canopen.RemoteNode(self.node_id, self.eds_path)
        self.network.add_node(self.node)

        # Read PDO configuration from device so mapping is known before first PDO write/read
        try:
            self.node.tpdo.read()
        except Exception as e:
            log.debug("TPDO read failed: %s", e)
        try:
            self.node.rpdo.read()
        except Exception as e:
            log.debug("RPDO read failed: %s", e)

        try:
            self.controlword_val = int(self._sdo_read(CONTROLWORD_INDEX, CONTROLWORD_SUB))
        except Exception:
            pass

        try:
            self.ModeofOperation_val = int(self._sdo_read(MODES_OF_OPERATION_INDEX, MODES_OF_OPERATION_SUB))
        except Exception:
            pass

        try:
            self.targetiq_val = float(self._sdo_read(TARGET_IQ_INDEX, TARGET_IQ_SUB))
        except Exception:
            pass

        # Iq gains (REAL32)
        try:
            self.iq_kp_val = float(self._sdo_read(IQ_KP_INDEX, IQ_KP_SUB))
        except Exception:
            pass
        try:
            self.iq_ki_val = float(self._sdo_read(IQ_KI_INDEX, IQ_KI_SUB))
        except Exception:
            pass

        # Speed gains (UNSIGNED32 according to your EDS)
        try:
            self.speedkp_val = int(self._sdo_read(SPEED_KP_INDEX, SPEED_KP_SUB))
        except Exception:
            pass
        try:
            self.speedki_cal = int(self._sdo_read(SPEED_KI_INDEX, SPEED_KI_SUB))
        except Exception:
            pass
        try:
            self.speedkd_val = int(self._sdo_read(SPEED_KD_INDEX, SPEED_KD_SUB))
        except Exception:
            pass
        try:
            self.speedTC_val = int(self._sdo_read(SPEED_TC_INDEX, SPEED_TC_SUB))
        except Exception:
            pass

        # Position gains (UNSIGNED16 in your EDS)
        try:
            self.positionkp_val = int(self._sdo_read(POS_KP_INDEX, POS_KP_SUB))
        except Exception:
            pass
        try:
            self.positionki_val = int(self._sdo_read(POS_KI_INDEX, POS_KI_SUB))
        except Exception:
            pass
        try:
            self.positionkd_val = int(self._sdo_read(POS_KD_INDEX, POS_KD_SUB))
        except Exception:
            pass
        try:
            self.positionTc_val = int(self._sdo_read(POS_TC_INDEX, POS_TC_SUB))
        except Exception:
            pass

        # NEW: set up TPDO callbacks for feedback
        try:
            self._setup_tpdo_callbacks()
        except Exception as e:
            log.debug("Failed to set up TPDO callbacks: %s", e)

    def disconnect(self) -> None:
        if self.network and self.node:
            try:
                self.network.remove_node(self.node_id)
            except Exception:
                pass
        if self.network:
            self.network.disconnect()
        self.network = None
        self.node = None

    # ---- NMT ----
    @property
    def state(self) -> str:
        self._ensure_node()
        return str(self.node.nmt.state)

    def to_operational(self) -> None:
        self._ensure_node()
        self.node.nmt.state = "OPERATIONAL"

    def to_preoperational(self) -> None:
        self._ensure_node()
        self.node.nmt.state = "PRE-OPERATIONAL"

    # ---- API (indices are direct from constants) ----
    # 1) Heartbeat
    def set_heartbeat(self, producer_time_ms: int) -> None:
        self._ensure_node()
        self._sdo_write(HEARTBEAT_INDEX, HEARTBEAT_SUB, int(producer_time_ms))

    # 3) Controlword
    def set_controlword(self, value: int = 0x000F, *, force_pdo: bool = False) -> None:
        self._ensure_node()
        
        
        nmt = str(self.node.nmt.state).upper()
        if nmt == "OPERATIONAL" and force_pdo:
            try:
                rpdo = self.node.pdo.rx[1]
                rpdo['Controlword'].raw = value
                self.controlword_val = value
                rpdo['modes_of_operation'].raw = self.ModeofOperation_val
                rpdo['target_iq'].raw = self.targetiq_val
                rpdo.transmit()
                return
            except LookupError:
                # Attempt a mapping refresh once, then retry
                self.refresh_pdo_cache()
                try:
                    rpdo = self.node.pdo.rx[1]
                    rpdo['Controlword'].raw = value
                    self.controlword_val = value
                    rpdo['modes_of_operation'].raw = self.ModeofOperation_val
                    rpdo['target_iq'].raw = self.targetiq_val
                    rpdo.transmit()
                    return
                except LookupError:
                    if force_pdo:
                        raise
        self._sdo_write(CONTROLWORD_INDEX, CONTROLWORD_SUB, value)
        self.controlword_val = value
    # 4) Iq gains
    def set_iq_gains(self, kp: Optional[float] = None, ki: Optional[float] = None) -> None:
        self._ensure_node()
        print(self._find_rpdo_for_object(IQ_KP_INDEX))
        nmt = str(self.node.nmt.state).upper()
        self.refresh_pdo_cache()
        if nmt == "OPERATIONAL" :
            rpdo = self.node.pdo.rx[2]
            if kp is not None:
                rpdo['iq_kp'].raw = kp
                self.iq_kp_val = kp
            else:
                rpdo['iq_kp'].raw = self.iq_kp_val

            if ki is not None:
                rpdo['iq_ki'].raw = ki
                self.iq_ki_val = ki
            else:
                rpdo['iq_ki'].raw = self.iq_ki_val
            rpdo.transmit()
        else :
            if kp is not None:
                self._sdo_write(IQ_KP_INDEX,IQ_KP_SUB,kp)
                self.iq_kp_val = kp
            if ki is not None:
                self._sdo_write(IQ_KI_INDEX,IQ_KI_SUB,ki)
                self.iq_ki_val = ki
    # 5) Velocity gains
    def set_velocity_gains(self, kp: Optional[float] = None, ki: Optional[float] = None,
                           kd: Optional[float] = None, Tc: Optional[float] = None) -> None:
        self._ensure_node()
        nmt = str(self.node.nmt.state).upper()
        if(nmt == "OPERATIONAL"):
            
            rpdo = self.node.pdo.rx[3]
            if kp is not None:
                rpdo['speed_kp'].raw = kp
                self.speedkp_val = kp
            else :
                rpdo['speed_kp'].raw = self.speedkp_val 
            
            if ki is not None:
                rpdo['speed_ki'].raw = ki
                self.speedki_cal = ki
            else :
                rpdo['speed_ki'].raw = self.speedki_cal
            
            if kd is not None:
                rpdo['speed_kd'].raw = kd
                self.speedkd_val = kd
            else:
                rpdo['speed_kd'].raw = self.speedkd_val
            
            if Tc is not None:
                rpdo['speed_Tc'].raw = Tc
                self.speedTC_val = Tc
            else:
                rpdo['speed_Tc'].raw = self.speedTC_val
            rpdo.transmit()
            print("wrote velocity gains")
            return
        if kp is not None and 'SPEED_KP_INDEX' in globals():
            self._sdo_write(SPEED_KP_INDEX, SPEED_KP_SUB, kp)
            self.speedkp_val = kp
        if ki is not None and 'SPEED_KI_INDEX' in globals():
            self._sdo_write(SPEED_KI_INDEX, SPEED_KI_SUB, ki)
            self.speedki_cal = ki
        if kd is not None and 'SPEED_KD_INDEX' in globals():
            self._sdo_write(SPEED_KD_INDEX, SPEED_KD_SUB, kd)
            self.speedkd_val = kd
        if Tc is not None and 'SPEED_TC_INDEX' in globals():
            self._sdo_write(SPEED_TC_INDEX, SPEED_TC_SUB, Tc)
            self.speedTC_val = Tc

    # 6) Position gains
    def set_position_gains(self, kp: Optional[float] = None, ki: Optional[float] = None,
                           kd: Optional[float] = None, Tc: Optional[float] = None) -> None:
        self._ensure_node()
        nmt = str(self.node.nmt.state).upper()
        if(nmt == "OPERATIONAL"):
            rpdo = self.node.pdo.rx[4]
            if kp is not None:
                rpdo['pos_kp'].raw = kp
                self.positionkp_val = kp
            else :
                rpdo['pos_kp'].raw = self.positionkp_val 
            
            if ki is not None:
                rpdo['pos_ki'].raw = ki
                self.positionki_val = ki
            else :
                rpdo['pos_ki'].raw = self.positionki_val 
            
            if kd is not None:
                rpdo['pos_kd'].raw = kd
                self.positionkd_val = kd
            else:
                rpdo['pos_kd'].raw = self.positionkd_val
            
            if Tc is not None:
                rpdo['pos_Tc'].raw = Tc
                self.positionTc_val = Tc
            else:
                rpdo['pos_Tc'].raw = self.positionTc_val
            rpdo.transmit()
            return

        if kp is not None and 'POS_KP_INDEX' in globals():
            self._sdo_write(POS_KP_INDEX, POS_KP_SUB, kp)
            self.positionkp_val = kp
        if ki is not None and 'POS_KI_INDEX' in globals():
            self._sdo_write(POS_KI_INDEX, POS_KI_SUB, ki)
            self.positionki_val = ki
        if kd is not None and 'POS_KD_INDEX' in globals():
            self._sdo_write(POS_KD_INDEX, POS_KD_SUB, kd)
            self.positionkd_val = kd
        if Tc is not None and 'POS_TC_INDEX' in globals():
            self._sdo_write(POS_TC_INDEX, POS_TC_SUB, Tc)
            self.positionTc_val = Tc

    # 7) Mode of operation
    def set_mode_of_operation(self, mode: int) -> None:
        self._ensure_node()
        nmt = str(self.node.nmt.state).upper()
        if nmt == "OPERATIONAL":
            try:
                rpdo = self.node.pdo.rx[1]
                rpdo['Controlword'].raw = self.controlword_val
                rpdo['modes_of_operation'].raw = mode
                self.ModeofOperation_val = mode
                rpdo['target_iq'].raw = self.targetiq_val
                rpdo.transmit()
                return
            except LookupError:
                # Attempt a mapping refresh once, then retry
                self.refresh_pdo_cache()
                try:
                    rpdo = self.node.pdo.rx[1]
                    rpdo['Controlword'].raw = self.controlword_val
                    self.ModeofOperation_val = mode
                    rpdo['modes_of_operation'].raw = mode
                    rpdo['target_iq'].raw = self.targetiq_val
                    rpdo.transmit()
                    return
                except :
                    print("Not able to set Mode of Operation")
        self._sdo_write(MODES_OF_OPERATION_INDEX, MODES_OF_OPERATION_SUB, mode)
        self.ModeofOperation_val = mode


    # 8) Target Iq
    def set_target_iq(self, iq: float, *, force_pdo: bool = True) -> None:
        self._ensure_node()
        nmt = str(self.node.nmt.state).upper()
        if nmt == "OPERATIONAL" and force_pdo:
            try:
                rpdo = self.node.pdo.rx[1]
                rpdo['Controlword'].raw = self.controlword_val
                rpdo['modes_of_operation'].raw = self.ModeofOperation_val
                rpdo['target_iq'].raw = iq
                self.targetiq_val = iq
                rpdo.transmit()
                return
            except LookupError:
                # Attempt a mapping refresh once, then retry
                self.refresh_pdo_cache()
                try:
                    rpdo = self.node.pdo.rx[1]
                    rpdo['Controlword'].raw = self.controlword_val
                    rpdo['modes_of_operation'].raw = self.ModeofOperation_val
                    rpdo['target_iq'].raw = iq
                    self.targetiq_val = iq
                    rpdo.transmit()
                    return
                except :
                    print("Not able to set target Iq")    
        self._sdo_write(TARGET_IQ_INDEX, TARGET_IQ_SUB, iq)
        self.targetiq_val = iq

    # 9) Target position + velocity
    def set_target_position_and_velocity(
        self,
        position: float,
        velocity: Optional[float] = None,
        *,
        force_pdo: bool = True
    ) -> None:
        self._ensure_node()
        if 'TARGET_POSITION_INDEX' in globals() and 'TARGET_VELOCITY_INDEX' in globals():
            wrote_both = self._try_write_same_pdo([
                (TARGET_POSITION_INDEX, TARGET_POSITION_SUB, position),
                (TARGET_VELOCITY_INDEX, TARGET_VELOCITY_SUB, velocity),
            ])
            if not wrote_both:
                self._smart_write(TARGET_POSITION_INDEX, TARGET_POSITION_SUB, position, prefer_pdo=True, force_pdo=force_pdo)
                if velocity is not None:
                    self._smart_write(TARGET_VELOCITY_INDEX, TARGET_VELOCITY_SUB, velocity, prefer_pdo=True, force_pdo=force_pdo)

    # 10) Velocity
    def set_velocity(self, velocity: float, *, force_pdo: bool = True) -> None:
        self._ensure_node()
        if(self.ModeofOperation_val == 3):
            self._smart_write(TARGET_VELOCITY_INDEX, TARGET_VELOCITY_SUB, velocity, prefer_pdo=True, force_pdo=force_pdo)

    # 11) Accel/Decel
    def set_accel_decel(self, accel: Optional[float] = 500, decel: Optional[float] = 500, *, prefer_pdo: bool = True) -> None:
        self._ensure_node()
        if 'TARGET_ACCELERATION_INDEX' in globals() and 'TARGET_DECELERATION_INDEX' in globals() and prefer_pdo:
            wrote_both = self._try_write_same_pdo([
                (TARGET_ACCELERATION_INDEX, TARGET_ACCELERATION_SUB, accel),
                (TARGET_DECELERATION_INDEX, TARGET_DECELERATION_SUB, decel),
            ])
        else:
            self._sdo_write(TARGET_ACCELERATION_INDEX, TARGET_ACCELERATION_SUB, accel)
            self._sdo_write(TARGET_DECELERATION_INDEX, TARGET_DECELERATION_SUB, decel)

    # ---- internals ----
    def _ensure_node(self) -> None:
        if not self.node or not self.network:
            raise RuntimeError("Not connected. Call .connect() first.")

    def _sdo_read(self, index: int, subindex: int = 0):
        """Read a value via SDO. Prefer phys, fall back to raw."""
        self._ensure_node()
        var = self.node.sdo[index]
        try:
            # If it's an array/record, indexing will work
            var = var[subindex]
        except TypeError:
            # Plain VAR: ignore subindex
            pass

        try:
            return var.phys
        except Exception:
            return var.raw


    def _sdo_write(self, index: int, subindex: int, value) -> None:
        """Handle both VAR (no subindex) and array/record objects."""
        assert self.node is not None
        var = self.node.sdo[index]
        try:
            # If it's an array/record, subscript will succeed
            var = var[subindex]
        except TypeError:
            # Plain VAR: ignore subindex
            pass
        # Prefer 'phys' if available and value is float; else raw
        try:
            var.phys = value
        except Exception:
            var.raw = value

    def refresh_pdo_cache(self) -> None:
        """Re-read PDO mapping/comm params from device."""
        assert self.node is not None
        try:
            self.node.tpdo.read()
        except Exception as e:
            log.debug("TPDO read failed (refresh): %s", e)
        try:
            self.node.rpdo.read()
        except Exception as e:
            log.debug("RPDO read failed (refresh): %s", e)

    def _smart_write(self, index: int, subindex: int, value, *, prefer_pdo: bool, force_pdo: bool = False) -> None:
        assert self.node is not None
        nmt = str(self.node.nmt.state).upper()
        if nmt == "OPERATIONAL" and prefer_pdo:
            try:
                self._pdo_write_by_index(index, subindex, value)
                return
            except LookupError:
                # Attempt a mapping refresh once, then retry
                self.refresh_pdo_cache()
                try:
                    self._pdo_write_by_index(index, subindex, value)
                    return
                except LookupError:
                    if force_pdo:
                        raise
        else:
            self._sdo_write(index, subindex, value)

    def _pdo_write_by_index(self, index: int, subindex: int, value) -> None:
        """Find an RPDO-mapped variable and transmit it. Raises LookupError if not mapped."""
        assert self.node is not None
        for _, rpdo in self.node.rpdo.items():
            for var in rpdo:
                if var.index == index and var.subindex == subindex:
                    # Prefer phys if available
                    try:
                        var.phys = value
                    except Exception:
                        var.raw = value
                    rpdo.transmit()
                    return
        raise LookupError(f"0x{index:04X}:{subindex} not mapped to any RPDO")

    def _try_write_same_pdo(self, items: List[Tuple[int, int, Optional[float]]]) -> bool:
        """Attempt to write multiple indices mapped in the same RPDO and transmit once."""
        assert self.node is not None
        valid_items = [(i, s, v) for (i, s, v) in items if v is not None]
        if not valid_items:
            return False
        for _, rpdo in self.node.rpdo.items():
            idxs = {(var.index, var.subindex) for var in rpdo}
            if all((i, s) in idxs for (i, s, _) in valid_items):
                # Set all mapped vars then single transmit
                for (i, s, v) in valid_items:
                    for var in rpdo:
                        if var.index == i and var.subindex == s:
                            try:
                                var.phys = v
                            except Exception:
                                var.raw = v
                            break
                rpdo.transmit()
                return True
        return False

    def _find_rpdo_for_object(self, index: int, subindex: int = 0):
        """
        Search all RPDOs (node.pdo.rx) for the given (index, subindex).
        Returns: (pdo, mapped_entry) or (None, None) if not found.
        """
        # node.pdo.rx is a dict-like: {pdo_num: PdoMap}
        try:
            rpdo_dict = self.node.pdo.rx
        except AttributeError:
            # Very old python-canopen or unexpected structure
            return (None, None)

        for pdo_num, pdo in rpdo_dict.items():
            # pdo.map is an iterable of mapped variables
            try:
                for mapped in pdo.map:
                    # mapped has .index and .subindex (and .name)
                    if mapped.index == index and mapped.subindex == subindex:
                        return (pdo, mapped)
            except Exception:
                # If pdo.map behaves unexpectedly, just skip this PDO
                continue

        return (None, None)

        # ---- TPDO feedback handling ----
    def _setup_tpdo_callbacks(self) -> None:
        """
        Register callbacks for TPDO1..3 so that feedback variables are kept
        up to date as CANopen TPDOs arrive.
        """
        self._ensure_node()

        # TPDO1: position_measured (0x6064, REAL64)
        try:
            tpdo1 = self.node.tpdo[1]
            tpdo1.add_callback(self._on_tpdo1)
        except Exception as e:
            log.debug("Unable to register TPDO1 callback: %s", e)

        # TPDO2: velocity_measured (0x606C, REAL32)
        try:
            tpdo2 = self.node.tpdo[2]
            tpdo2.add_callback(self._on_tpdo2)
        except Exception as e:
            log.debug("Unable to register TPDO2 callback: %s", e)

        # TPDO3: iq_measured, id_measured (0x2032, 0x2033, REAL32)
        try:
            tpdo3 = self.node.tpdo[3]
            tpdo3.add_callback(self._on_tpdo3)
        except Exception as e:
            log.debug("Unable to register TPDO3 callback: %s", e)

    def _on_tpdo1(self, tpdo) -> None:
        """
        TPDO1 – position feedback.
        Mapping (0x1A00): 0x6064 position_measured (REAL64).
        """
        try:
            self.position_measured = float(tpdo['position_measured'].phys)
            self.new_state_available = True
        except Exception as e:
            log.debug("Failed to parse TPDO1 (position_measured): %s", e)

    def _on_tpdo2(self, tpdo) -> None:
        """
        TPDO2 – velocity feedback.
        Mapping (0x1A01): 0x606C velocity_measured (REAL32).
        """
        try:
            self.velocity_measured = float(tpdo['velocity_measured'].phys)
            self.new_state_available = True
        except Exception as e:
            log.debug("Failed to parse TPDO2 (velocity_measured): %s", e)

    def _on_tpdo3(self, tpdo):
        try:
            self.iq_measured = float(tpdo['iq_measured'].phys)
            self.id_measured = float(tpdo['id_measured'].phys)
            self.new_state_available = True
        except Exception as e:
            log.debug("Failed to parse TPDO3: %s", e)

    def get_motor_state(self) -> dict:
        """
        Return the current motor state consisting of:
        - position_measured
        - velocity_measured
        - iq_measured
        - id_measured
        - new: indicates if new TPDO data has arrived since the last call
        """

        state = {
            "position": self.position_measured,
            "velocity": self.velocity_measured,
            "iq": self.iq_measured,
            "id": self.id_measured,
            "new": self.new_state_available,
        }

        # Reset the flag after reading
        self.new_state_available = False

        return state




