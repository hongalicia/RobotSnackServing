TCP protocol summary:

- TCP
- Packet length: fixed 8 bytes
- Header: 0x88 0x66
- Checksum: sum(packet[0..7]) & 0xFF == 0

Command:
- 'f' (0x66): Lid status

Payload:
- data[3] = lid_id
    - 1 → 1st lid
    - 2 → 2nd lid
- data[4] = state
    - 0 → lid open
    - 1 → lid closed

Examples:

1st lid closed:
88 66 66 01 01 00 00 XX

2nd lid open:
88 66 66 02 00 00 00 XX

(XX = checksum)

On our side, we just call:
- get_1st_isClose()
- get_2nd_isClose()

to read the latest lid status.
