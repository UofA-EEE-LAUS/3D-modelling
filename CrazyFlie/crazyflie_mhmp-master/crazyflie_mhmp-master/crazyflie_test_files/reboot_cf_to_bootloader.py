from crazyradio import Crazyradio
import time
cr = Crazyradio()

cr.set_channel(100)
cr.set_data_rate(cr.DR_250KPS)

print(cr.send_packet((0xff, 0xfe, 0xff)).data)   # Init the reboot

# print(cr.send_packet((0xff, 0xfe, 0xf0, 0)).data)   # Reboot to Bootloader
print(cr.send_packet((0xff, 0xfe, 0xf0, 1)).ack)   # Reboot to Firmware