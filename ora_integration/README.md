# ora_integration

## Set up the virtual CAN interface

- `sudo modprobe vcan`
- `sudo ip link add dev vcan0 type vcan`
- `sudo ip link set up vcan0`

## Make some CAN messages

- `sudo apt install can-utils`
- To generate random messages: `cangen vcan0`
- To display them: `candump vcan0`