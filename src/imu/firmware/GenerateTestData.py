import numpy as np

vnoise = 2.5e-4
tnoise = 1e-5
vg = 9.81*2.5e-3

def v2hex(v, exp=14):
    w = round(v*2**exp)
    if w < 0:
        w += 2**16
    assert 0<=w<2**16
    return '0x{:04x}'.format(w)

def packetstr(vx, vy, vz, tx, ty, tz):
    dv = list(map(v2hex, (vx, vy, vz)))
    dt = list(map(lambda v: v2hex(v, exp=19), (tx, ty, tz)))
    return ('{0, '+', '.join(['{}']*6+['0']*9).format(*dv, *dt)+'}')

init_packet_count = 80
still_packet_count = 720
rx_packet_count = 10000

dv = np.array([[vg*1.3, vg*1.3, vg*1.3]]*init_packet_count)
dv += np.random.randn(*dv.shape)*vnoise
dt = np.array([[0, 0, 0.0]]*init_packet_count)
dt += np.random.randn(*dt.shape)*tnoise
initwords = '{\n'+',\n'.join(packetstr(*x, *dt[i]) for i, x in enumerate(dv))+'};'

dv = np.array([[0, 0, vg]]*still_packet_count)
dv += np.random.randn(*dv.shape)*vnoise
dt = np.array([[0, 0, 0.0]]*still_packet_count)
dt += np.random.randn(*dt.shape)*tnoise
stillwords = '{\n'+',\n'.join(packetstr(*x, *dt[i]) for i, x in enumerate(dv))+'};'

dv = np.array([[0.01, 0.05, 0.1+vg]]*400+[[0, 0, 0]]*(rx_packet_count-400))
dv += np.random.randn(*dv.shape)*vnoise
dt = np.array([[0.001, 0.001, 0.002]]*400+[[0.005, 0.005, 0.01]]*(rx_packet_count-400))
dt += np.random.randn(*dt.shape)*tnoise
rxwords = '{\n'+',\n'.join(packetstr(*x, *dt[i]) for i, x in enumerate(dv))+'};'

tstr = """#define INITPACKETCOUNT {}
#define STILLPACKETCOUNT {}
#define RXPACKETCOUNT {}
uint16_t initwords[][16] = {}
uint16_t stillwords[][16] = {}
uint16_t rxwords[][16] = {}
""".format(init_packet_count, still_packet_count, rx_packet_count,
           initwords, stillwords, rxwords)

with open('RxData.h', 'w') as file:
    file.write(tstr)
