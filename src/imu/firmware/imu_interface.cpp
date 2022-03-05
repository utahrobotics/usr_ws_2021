#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "Arduino.h"
#include "util.h"

#define NO_CRC_CHECK

#ifndef NO_CRC_CHECK
#define BADCRC 16
#endif

/* pin definitions */
#define ClkPin 6
#define InFrame 9
#define DataReady 10
#define SPin 11
#define ClkIntEnabled 12
#define IMUInit 13 /* Orange LED indicates when IMU is ready */
#define BADCHECKSUM 14
#define BADDATA 15
#define IMUDataReady 17
#define RxData 20

#define OpeningFlag 0x7e
#define HDLC_FRAME_LENGTH 14
#define ACC_BUFFER_LENGTH 400

#define SETIMUINIT(v) digitalWriteFast(IMUInit, v); imuInit = (bool) v
#define SETINFRAME(v) digitalWriteFast(InFrame, v); inFrame = (bool) v
#define SETREADY(v) digitalWriteFast(DataReady, v); ready = (bool) v
#define SETSPIN(v) digitalWriteFast(SPin, v); spin = (bool) v
#define DETACHCLK if (interrupt) {\
    interrupt = false;\
    digitalWriteFast(ClkIntEnabled, LOW);\
}

ros::NodeHandle nh;

std_msgs::String debug_msg;
/* Stores IMU data in the /imu frame. Includes gravity. */
/* Note that imu_msg.orientation is always zero. */
/* This is because in the /imu frame, the angle doesn't change. */
sensor_msgs::Imu imu_msg;
/* Stores current positional and angular velocities in the /imu frame. */
/* Offset for gravity. */
geometry_msgs::TwistStamped vel_msg;
/* This keeps track of IMU orientation and position. */
/* Used to transform between /imu and /imu_base frames. */
/* /imu_base frame is related to the world frame by a static transform. */
geometry_msgs::TransformStamped transform;

// TODO: double check that this is always zero initialized
static Vector3 accbuffer[ACC_BUFFER_LENGTH];
int accidx = 0;
int initidx = -1;
/* Gravity in /imu_base frame */
Vector3 gravity_base;
/* Gravity in /imu frame */
Vector3 gravity;
/* Velocity in /imu_base frame */
Vector3 velocity;

ros::Publisher debug_pub("debug", &debug_msg);
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher vel_pub("vel", &vel_msg);
tf::TransformBroadcaster tfbroadcaster;

double ts; /* time stamp set when IMUDataReady drops */
uint16_t buffer[HDLC_FRAME_LENGTH];
uint16_t buffer2[HDLC_FRAME_LENGTH+1];
uint16_t rawbuffer[17];
uint16_t derBuffer = 0;
uint16_t wordBuffer = 0;
bool imuInit = false;
bool inFrame = false;
bool ready = false;
bool spin = true;
bool interrupt = false;
int bitCounter = 0;
int rawBitCounter = 0;
int idx = HDLC_FRAME_LENGTH;

#include "RxData.h"
static int rxpacketidx = -1-(INITPACKETCOUNT+STILLPACKETCOUNT);
static int rxidx = -1;
static uint8_t rxbuffer = 0;

inline void setchecksum(uint16_t* buffer) {
    buffer[0] = OpeningFlag << 8u;
    uint16_t sum = 0;
    for (size_t i = 1; i < 13; i++) {
        sum += buffer[i];
    }
    buffer[13] = (uint16_t) ~sum;
    buffer[15] = OpeningFlag;
}

uint16_t* wordbuffer;

inline uint8_t readNextRxDataBit() {
//    uint16_t* wordbuffer;
    if (rxpacketidx < -STILLPACKETCOUNT) {
        wordbuffer = &initwords[rxpacketidx+INITPACKETCOUNT+STILLPACKETCOUNT][0];
    }
    else if (rxpacketidx < 0) {
        wordbuffer = &stillwords[rxpacketidx+STILLPACKETCOUNT][0];
    }
    else {
        wordbuffer = &rxwords[rxpacketidx][0];
    }

    if (++rxidx < 0) {
        return 0;
    }
    rxbuffer <<= 1u;
    rxbuffer |= (wordbuffer[rxidx / 16] >> (rxidx % 16)) & 1u;

    /* Add padding bit if not in a flag word */
    if (!((rxbuffer & 0x3e) ^ 0x3e) && rxidx >= 16 && rxidx < 16*15) {
        rxidx--;
        rxbuffer &= ~1u;
    }
    uint8_t value = rxbuffer & 1u;
    digitalWriteFast(RxData, value);
    return value;
}

static double tclk_ISR[1000];
static int tclk_ISRidx = 0;

/* IMU clock interrupt */
void clk_ISR(void) {
    /* If we already have data, leave */
    if (ready) {
        /* but first detach interrupt */
        DETACHCLK
        return;
    }
    register uint8_t rval = readNextRxDataBit();
    derBuffer <<= 1;
    derBuffer |= rval;
    /* look for opening/closing flag */
    if ((derBuffer & 0xff) == OpeningFlag) {
        inFrame = !inFrame;
        digitalWriteFast(InFrame, inFrame);
        if (inFrame) {
            rawBitCounter = 0;
            bitCounter = 0;
            idx = 0;
        }
        else {
            digitalWriteFast(BADDATA, (idx == HDLC_FRAME_LENGTH ? LOW : HIGH));
            SETREADY(1);
        }
        return;
    }

    if (!inFrame) {
        return;
    }
    else {
        rawbuffer[rawBitCounter / 16] <<= 1;
        rawbuffer[rawBitCounter++ / 16] |= rval;
    }

    /* add bits to wordbuffer and store words to buffer */
    /* exclude padding bits */
    if ((idx == 0 && bitCounter < 5) || ((derBuffer & 0x3f) ^ 0x3e)) {
        wordBuffer <<= 1;
        wordBuffer |= rval;
        if (++bitCounter == 16) {
            bitCounter = 0;
            buffer2[idx] = reverse(wordBuffer);
            if (idx >= HDLC_FRAME_LENGTH) {
                /* Too much data, return what we have */
                /* Don't want to get stuck in the clock interrupt */
                /* Assume error in the opening/closing flag */
                digitalWriteFast(BADDATA, HIGH);
                SETREADY(1);
                return;
            }
            buffer[idx++] = reverse(wordBuffer);
        }
    }
}

/* IMU data ready interrupt */
void set_ts(void) {
    /* set timestamp and prepare to receive data */
    ts = seconds();
    SETINFRAME(0);
    SETREADY(0);
    SETSPIN(0);
    digitalWriteFast(BADDATA, LOW);
    digitalWriteFast(BADCHECKSUM, LOW);
#ifndef NO_CRC_CHECK
    digitalWriteFast(BADCRC, LOW);
#endif
}

struct ImuData {
    double timestamp;
    int16_t x_delta_vel;
    int16_t y_delta_vel;
    int16_t z_delta_vel;
    int16_t x_delta_angle;
    int16_t y_delta_angle;
    int16_t z_delta_angle;
    uint16_t imu_status_summary_word;
    uint16_t mux_id;
    uint16_t multiplexed_data_word;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
    uint16_t checksum;

    ImuData() : timestamp(0),
                x_delta_vel(0),
                y_delta_vel(0),
                z_delta_vel(0),
                x_delta_angle(0),
                y_delta_angle(0),
                z_delta_angle(0),
                imu_status_summary_word(0),
                mux_id(0),
                multiplexed_data_word(0),
                reserved1(0),
                reserved2(0),
                reserved3(0),
                checksum(0xffff)
    {}

    /* assign data from the buffer if CRC and checksum are good */
    /* otherwise only timestamp is updated */
    /* negative rv if CRC is wrong, positive if data sum is wrong */
    int set_data(const uint16_t* data) {
        timestamp = ts;
        uint16_t csdiff;
#ifndef NO_CRC_CHECK
        csdiff = crc_check(data, 13);
        if (csdiff) {
            digitalWriteFast(BADCRC, HIGH);
            return -((int) csdiff);
        }
#endif
        uint16_t sum = 0;
        for (size_t i = 0; i < 12; i++) {
            sum += data[i];
        }
        csdiff = ((uint16_t) ~sum) ^ data[12];
        if (csdiff) {
            digitalWriteFast(BADCHECKSUM, HIGH);
            return csdiff;
        }
        x_delta_vel = (int16_t) data[0];
        y_delta_vel = (int16_t) data[1];
        z_delta_vel = (int16_t) data[2];
        x_delta_angle = (int16_t) data[3];
        y_delta_angle = (int16_t) data[4];
        z_delta_angle = (int16_t) data[5];
        imu_status_summary_word = data[6];
        mux_id = data[7];
        multiplexed_data_word = data[8];
        reserved1 = data[9];
        reserved2 = data[10];
        reserved3 = data[11];
        checksum = data[12];
        return 0;
    }
} raw_imu_data, raw_imu_data1;

#define DELTA(dir, q) (raw_imu_data1.dir##_delta_##q)
#define DTHETA(dir) ldexp((double) DELTA(dir, angle), ANGLE_EXP)
#define DV(dir) ldexp((double) DELTA(dir, vel), VEL_EXP)
#define DT (raw_imu_data1.timestamp - raw_imu_data.timestamp)
#define DIFF(dir, q) (DELTA(dir, q) / DT)
#define ANGLE_EXP -19
#define VEL_EXP -14

inline int fillImuMsg() {
    raw_imu_data = raw_imu_data1;
    int status = raw_imu_data1.set_data(buffer);
    // TODO: report if nonzero status
    imu_msg.header.stamp = nh.then(raw_imu_data1.timestamp);

    vel_msg.header.stamp = imu_msg.header.stamp;
    transform.header.stamp = imu_msg.header.stamp;

    Vector3 delta_angle = v3(DTHETA(x), DTHETA(y), DTHETA(z));
    Vector3 delta_vel = v3(DV(x), DV(y), DV(z));

    if (!imuInit) {/* Not initialized, collect samples of gravity */
        accbuffer[accidx++] = (1/DT) * delta_vel;
        if (accidx >= ACC_BUFFER_LENGTH) {
            accidx = 0;
        }
    }

    // TODO: Possibly a better way to detect that IMU is initialized
    if (initidx < 0) {
        double g = magnitude(
                vavg(accbuffer, accidx - 3, accidx, ACC_BUFFER_LENGTH));

        if (9.6 < g && g < 10) {
            initidx = accidx;
        }
        /* If IMU is not initialized, do not publish data */
        return 1;
    }
    else if (!imuInit) {
        if (accidx == initidx) {
            gravity_base = vavg(accbuffer, 0, ACC_BUFFER_LENGTH, ACC_BUFFER_LENGTH);
            imu_msg.linear_acceleration = gravity_base;
            SETIMUINIT(1);
        }
        else {/* Initialized, but gravity_base not set */
            return -1;
        }
    }

    /* Estimate Dθ (ω) and Dv (a) */
    imu_msg.angular_velocity = (1/DT) * delta_angle;
    imu_msg.linear_acceleration = (1/DT) * delta_vel;

    Vector3 old_gravity = gravity;
    transform.transform.rotation += 0.5 * transform.transform.rotation * v2q(delta_angle);
    gravity = crotate(gravity_base, transform.transform.rotation);

    vel_msg.twist.angular = imu_msg.angular_velocity;
    /* Offset for gravity based on trapezoid rule */
    vel_msg.twist.linear += delta_vel - (DT/2) * (old_gravity + gravity);

    Vector3 old_velocity = velocity;
    velocity = rotate(vel_msg.twist.linear, transform.transform.rotation);

    /* Estimate ∫vdt using trapezoid rule */
    transform.transform.translation += (DT/2) * (old_velocity + velocity);

    return 0;
}

static double data_period = 2.5e-3;
static double next_clk_ts = 0;
static double clk_period = 1 / 1.015e6;

inline void delayUntilClkInt() {
    double ndelay = 1e9*(next_clk_ts-seconds());
    if (ndelay > 0) {
        delayNanoseconds((uint32_t) ndelay);
    }
    next_clk_ts += clk_period;
    digitalWriteFast(ClkPin, HIGH);
    double now = seconds();
    clk_ISR();
    double diff = seconds() - now;
    digitalWriteFast(ClkPin, LOW);
    tclk_ISR[tclk_ISRidx++] = diff;
    tclk_ISRidx %= 1000;
}

void setup() {
    Serial.begin(115200);

#ifndef NO_CRC_CHECK
    pinMode(BADCRC, OUTPUT);
    digitalWriteFast(BADCRC, LOW);
#endif

    pinMode(BADCHECKSUM, OUTPUT);
    pinMode(BADDATA, OUTPUT);
    pinMode(ClkPin, OUTPUT);
    pinMode(IMUDataReady, OUTPUT);
    pinMode(RxData, OUTPUT);
    pinMode(IMUInit, OUTPUT);
    pinMode(InFrame, OUTPUT);
    pinMode(DataReady, OUTPUT);
    pinMode(SPin, OUTPUT);
    pinMode(ClkIntEnabled, OUTPUT);

    digitalWriteFast(BADCHECKSUM, LOW);
    digitalWriteFast(BADDATA, LOW);
    SETIMUINIT(0);
    SETINFRAME(0);
    SETREADY(0);
    SETSPIN(1);
    digitalWriteFast(ClkIntEnabled, interrupt);
    digitalWriteFast(ClkPin, LOW);
    digitalWriteFast(IMUDataReady, HIGH);

    nh.initNode();
    nh.setSpinTimeout(1); // 1 ms timeout on spin
    nh.advertise(debug_pub);
    nh.advertise(imu_pub);
    nh.advertise(vel_pub);
    tfbroadcaster.init(nh);

    imu_msg.header.frame_id = "/imu";
    imu_msg.orientation.w = 1; // zero angle
    transform.transform.rotation.w = 1;

    // TODO: define known covariance
    for (size_t i = 0; i < 9; i++) {
        imu_msg.angular_velocity_covariance[i] = 0;
        imu_msg.linear_acceleration_covariance[i] = 0;
        imu_msg.orientation_covariance[i] = 0;
    }

    vel_msg.header.frame_id = "/imu";
    transform.header.frame_id = "/imu_base";
    transform.child_frame_id = "/imu";

    for (size_t i = 0; i < INITPACKETCOUNT; i++) {
        setchecksum(&initwords[i][0]);
    }
    for (size_t i = 0; i < STILLPACKETCOUNT; i++) {
        setchecksum(&stillwords[i][0]);
    }
    for (size_t i = 0; i < RXPACKETCOUNT; i++) {
        setchecksum(&rxwords[i][0]);
    }

    /* wait for connection before starting work */
    while (!nh.connected()) {
        nh.spinOnce();
    }

    debug_msg.data = "Finished setup";
    debug_pub.publish(&debug_msg);
    nh.spinOnce();

//    delay(100);

    ts = seconds();
}

static double tspin, tfill, tpdebug, tpimu, tpvel, tptf, tptotal;
static int lmsgdebug, lmsgimu, lmsgvel, lmsgtf, lmsgtotal;

void loop() {
    if (ready) {
        DETACHCLK

        int lmsgdbg = 0;

        double now = seconds();
        char debug_str[999];
        char buffer2_str[99];
        char wordbuffer_str[99];

        int off = 0;
        for (int i = 0; i < HDLC_FRAME_LENGTH; i++) {
            off += sprintf(&buffer2_str[off], "%04x", reverse(buffer2[i]));
        }

        off = 0;
        for (int i = 0; i < HDLC_FRAME_LENGTH;) {
            off += sprintf(&wordbuffer_str[off], "%04x", reverse(wordbuffer[++i]));
        }

        if (strcmp(buffer2_str, wordbuffer_str)) {
            sprintf(debug_str, "Sent: %s\nReceived: %s", wordbuffer_str, buffer2_str);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }

        if (buffer2[HDLC_FRAME_LENGTH]) {
            sprintf(debug_str, "idx: %d\nbitCounter: %d",
                    idx, bitCounter);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
            int off = sprintf(debug_str, "buffer2:   ");
            for (int i = 0; i <= HDLC_FRAME_LENGTH; i++) {
                off += sprintf(&debug_str[off], "%04x", reverse(buffer2[i]));
            }
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
            off = sprintf(debug_str, "wordbuffer:");
            for (int i = 0; i <= HDLC_FRAME_LENGTH;) {
                off += sprintf(&debug_str[off], "%04x", reverse(wordbuffer[++i]));
            }
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
            off = sprintf(debug_str, "rawbuffer: ");
            for (int i = 0; i < 17; i++) {
                off += sprintf(&debug_str[off], "%04x", rawbuffer[i]);
                rawbuffer[i] = 0;
            }
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
            buffer2[HDLC_FRAME_LENGTH] = 0;
        }

        if (abs(DT - data_period) > 10e-6) {
            sprintf(debug_str, "Last timestamp diff: %.3f ms", DT*1e3);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }

        if (tspin > 1e-3) {
            sprintf(debug_str, "Last spin: %.3f ms", tspin*1e3);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }

        double tdiffmin = 1, tdiffmax = 0;
        for (int i = 0; i < tclk_ISRidx; i++) {
            double tdiff = tclk_ISR[i];
            if (tdiff < tdiffmin) {
                tdiffmin = tdiff;
            }
            if (tdiff > tdiffmax) {
                tdiffmax = tdiff;
            }
        }

        if (tdiffmax > max(clk_period, 1.1e-6)) {
            sprintf(debug_str, "clk_ISR times: (%.3f us, %.3f us)",
                     tdiffmin*1e6, tdiffmax*1e6);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }

        if (tfill > 1e-3) {
            sprintf(debug_str, "Last fill: %.3f ms", tfill*1e3);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }

        tptotal = tpdebug + tpimu + tpvel + tptf;
        if (tptotal > 1e-3) {
            sprintf(debug_str,
                    "Last publish times:\n"
                    "    Debug (%d): %.3f ms\n"
                    "    IMU (%d): %.3f ms\n"
                    "    Twist (%d): %.3f ms\n"
                    "    Transform (%d): %.3f ms\n"
                    "    Total (%d): %.3f ms",
                    lmsgdebug, tpdebug*1e3, lmsgimu, tpimu*1e3,
                    lmsgvel, tpvel*1e3, lmsgtf, tptf*1e3,
                    lmsgtotal, tptotal*1e3);
            debug_msg.data = debug_str;
            lmsgdbg += debug_pub.publish(&debug_msg);
        }
        tpdebug = seconds() - now;

        now = seconds();
        int status = fillImuMsg();
        tfill = seconds() - now;
        if (status <= 0) {
            now = seconds();
            lmsgimu = imu_pub.publish(&imu_msg);
            tpimu = seconds() - now;
            if (!status) {/* Only publish if gravity is set */
                now = seconds();
                lmsgvel = vel_pub.publish(&vel_msg);
                tpvel = seconds() - now;
                now += tpvel;
                lmsgtf = tfbroadcaster.sendTransform(transform);
                tptf = seconds() - now;
            }
        }

        lmsgdebug = lmsgdbg;
        lmsgtotal = lmsgdebug + lmsgimu + lmsgvel + lmsgtf;
#ifndef NO_CRC_CHECK
        digitalWriteFast(BADCRC, LOW);
#endif
        digitalWriteFast(BADCHECKSUM, LOW);
        SETREADY(0);
        SETSPIN(1);
    }
    else if (spin) {
        /* Spin once, then wait for data ready */
        double now = seconds();
        nh.spinOnce();
        tspin = seconds() - now;
        double udelay = 1e6*(ts+data_period-seconds());
        if (udelay > 0) {
            delayMicroseconds((uint32_t) udelay);
        }
        digitalWriteFast(IMUDataReady, LOW);
        set_ts();
        delayMicroseconds(10);
        digitalWriteFast(IMUDataReady, HIGH);
    }
    else if (!interrupt) {
        tclk_ISRidx = 0;
        /* enable IMU clock interrupt */
        interrupt = true;
        digitalWriteFast(ClkIntEnabled, HIGH);
        if (++rxpacketidx >= RXPACKETCOUNT) {
            rxpacketidx = 0;
        }
        rxidx = -200;
        rxbuffer = 0;
        next_clk_ts = seconds() + clk_period;
    }
    else {
        /* wait for interrupt */
//        asm("wfi");
        /* but keep running CPU clock */
        delayUntilClkInt();
    }
}
