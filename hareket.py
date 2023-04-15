import can 
import time

def git(hiz):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
    msg = can.Message(arbitration_id=0x560, data = [1, 128, 138, 0], is_extended_id = False)
    while basla_flag:
        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}")
            sleep(0.05)
        except can.CanError:
            print("Message NOT sent")