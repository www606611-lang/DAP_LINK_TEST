#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "WireProtocol.h"

namespace {

constexpr char kAccessPointSsid[] = "CAR-K230";
constexpr char kAccessPointPassword[] = "K230CAR2026";
const IPAddress kAccessPointAddress(192, 168, 4, 1);

constexpr uint16_t kUdpListenPort = 4210;
constexpr int32_t kAccessPointChannel = 1;
constexpr uint32_t kChassisBaud = 115200;
constexpr int8_t kChassisRxPin = 20;
constexpr int8_t kChassisTxPin = 21;
constexpr uint32_t kPeerOfflineMs = 1500;
constexpr uint32_t kHeartbeatPeriodMs = 250;
constexpr uint32_t kStatsPeriodMs = 2000;
constexpr uint32_t kAccessPointRestartDelayMs = 500;
constexpr uint32_t kAccessPointRestartCooldownMs = 3000;
constexpr size_t kMaxUdpDatagram = 256;
constexpr size_t kMaxUartBytesPerLoop = 128;
constexpr uint8_t kMaxUdpPacketsPerLoop = 4;

HardwareSerial chassisUart(1);
WiFiUDP udp;
WireProtocol::Parser udpParser;
WireProtocol::Parser uartParser;

IPAddress peerAddress;
uint16_t peerPort = 0;
uint32_t lastPeerRxMs = 0;
uint32_t lastBridgeUartTxMs = 0;
uint32_t lastBridgeUdpTxMs = 0;
uint32_t lastStatsMs = 0;
uint16_t bridgeSequence = 0;
bool uartHelloSent = false;
bool udpHelloSent = false;
bool apConfigOk = false;
bool apStartOk = false;
bool udpStartOk = false;
volatile bool apRestartRequested = false;
volatile bool apRestartInProgress = false;
volatile uint32_t apRestartRequestMs = 0;
volatile uint32_t apDisconnectEvents = 0;
uint32_t apRestartCount = 0;
volatile uint32_t lastApRestartMs = 0;
float maxChipTemperatureC = -273.15F;
uint32_t temperatureReadErrors = 0;

struct BridgeStats {
    uint32_t udpDatagrams = 0;
    uint32_t udpOversize = 0;
    uint32_t udpToUart = 0;
    uint32_t bridgeToUart = 0;
    uint32_t uartToUdp = 0;
    uint32_t uartFramesWithoutPeer = 0;
    uint32_t udpSendErrors = 0;
    uint32_t uartShortWrites = 0;
} stats;

float sampleChipTemperatureC() {
    const float temperatureC = temperatureRead();
    if (temperatureC > -40.0F && temperatureC < 150.0F) {
        if (temperatureC > maxChipTemperatureC) {
            maxChipTemperatureC = temperatureC;
        }
        return temperatureC;
    }
    ++temperatureReadErrors;
    return -273.15F;
}

void startAccessPoint() {
    WiFi.mode(WIFI_AP);
    apStartOk = WiFi.softAP(kAccessPointSsid, kAccessPointPassword,
                            kAccessPointChannel, false, 4);
    apConfigOk = WiFi.softAPIP() == kAccessPointAddress;
    udpStartOk = apStartOk && udp.begin(kUdpListenPort) == 1;
}

void onWiFiEvent(arduino_event_id_t event) {
    if (event != ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
        return;
    }

    ++apDisconnectEvents;
    const uint32_t nowMs = millis();
    if (apRestartInProgress ||
        (lastApRestartMs != 0 &&
         static_cast<uint32_t>(nowMs - lastApRestartMs) <
             kAccessPointRestartCooldownMs)) {
        return;
    }
    apRestartRequestMs = nowMs;
    apRestartRequested = true;
}

void serviceAccessPointRecovery(uint32_t nowMs) {
    if (!apRestartRequested ||
        static_cast<uint32_t>(nowMs - apRestartRequestMs) <
            kAccessPointRestartDelayMs) {
        return;
    }

    apRestartInProgress = true;
    apRestartRequested = false;
    lastApRestartMs = nowMs;
    peerAddress = IPAddress();
    peerPort = 0;
    lastPeerRxMs = 0;
    udpHelloSent = false;
    udp.stop();
    WiFi.softAPdisconnect(true);
    delay(20);
    startAccessPoint();
    ++apRestartCount;
    apRestartInProgress = false;
}

bool peerOnline(uint32_t nowMs) {
    return peerPort != 0 && static_cast<uint32_t>(nowMs - lastPeerRxMs) <
                                kPeerOfflineMs;
}

size_t encode(const WireProtocol::Frame& frame, uint8_t* output,
              size_t capacity) {
    return WireProtocol::encodeFrame(frame.type, frame.sequence, frame.payload,
                                     frame.length, output, capacity);
}

void sendToChassis(const WireProtocol::Frame& frame) {
    uint8_t packet[WireProtocol::kMaxFrameSize];
    const size_t length = encode(frame, packet, sizeof(packet));
    if (length == 0 || chassisUart.write(packet, length) != length) {
        ++stats.uartShortWrites;
        return;
    }
    ++stats.udpToUart;
}

void sendToPeer(const WireProtocol::Frame& frame, uint32_t nowMs) {
    if (!peerOnline(nowMs)) {
        ++stats.uartFramesWithoutPeer;
        return;
    }

    uint8_t packet[WireProtocol::kMaxFrameSize];
    const size_t length = encode(frame, packet, sizeof(packet));
    if (length == 0 || udp.beginPacket(peerAddress, peerPort) != 1 ||
        udp.write(packet, length) != length || udp.endPacket() != 1) {
        ++stats.udpSendErrors;
        return;
    }
    ++stats.uartToUdp;
}

void emitBridgePresence(uint8_t type, bool toUart, bool toUdp,
                        uint32_t nowMs) {
    WireProtocol::Frame frame{};
    frame.type = type;
    frame.sequence = bridgeSequence++;
    frame.length = 5;
    frame.payload[0] = WireProtocol::kRoleEsp32;
    frame.payload[1] = static_cast<uint8_t>(nowMs);
    frame.payload[2] = static_cast<uint8_t>(nowMs >> 8);
    frame.payload[3] = static_cast<uint8_t>(nowMs >> 16);
    frame.payload[4] = static_cast<uint8_t>(nowMs >> 24);

    uint8_t packet[WireProtocol::kMaxFrameSize];
    const size_t length = encode(frame, packet, sizeof(packet));
    if (toUart) {
        if (length != 0 && chassisUart.write(packet, length) == length) {
            ++stats.bridgeToUart;
            lastBridgeUartTxMs = nowMs;
            uartHelloSent = true;
        } else {
            ++stats.uartShortWrites;
        }
    }
    if (toUdp) {
        lastBridgeUdpTxMs = nowMs;
        if (length == 0 || udp.beginPacket(peerAddress, peerPort) != 1 ||
            udp.write(packet, length) != length || udp.endPacket() != 1) {
            ++stats.udpSendErrors;
        } else {
            udpHelloSent = true;
        }
    }
}

void sendBridgePresence(uint32_t nowMs) {
    const bool online = peerOnline(nowMs);
    if (!online) {
        udpHelloSent = false;
    }
    const bool uartDue = static_cast<uint32_t>(
        nowMs - lastBridgeUartTxMs) >= kHeartbeatPeriodMs;
    const bool udpDue = online && static_cast<uint32_t>(
        nowMs - lastBridgeUdpTxMs) >= kHeartbeatPeriodMs;
    if (uartDue) {
        emitBridgePresence(uartHelloSent ? WireProtocol::kHeartbeat
                                         : WireProtocol::kHello,
                           true, false, nowMs);
    }
    if (udpDue) {
        emitBridgePresence(udpHelloSent ? WireProtocol::kHeartbeat
                                        : WireProtocol::kHello,
                           false, true, nowMs);
    }
}

void serviceUdp(uint32_t nowMs) {
    uint8_t datagram[kMaxUdpDatagram];
    for (uint8_t packetIndex = 0; packetIndex < kMaxUdpPacketsPerLoop;
         ++packetIndex) {
        const int packetSize = udp.parsePacket();
        if (packetSize <= 0) {
            return;
        }
        ++stats.udpDatagrams;

        if (packetSize > static_cast<int>(sizeof(datagram))) {
            ++stats.udpOversize;
            while (udp.available() > 0) {
                udp.read(datagram, sizeof(datagram));
            }
            continue;
        }

        const int received = udp.read(datagram, packetSize);
        if (received <= 0) {
            continue;
        }

        bool acceptedPeer = false;
        for (int index = 0; index < received; ++index) {
            WireProtocol::Frame frame{};
            if (udpParser.feed(datagram[index], frame)) {
                if (!acceptedPeer) {
                    peerAddress = udp.remoteIP();
                    peerPort = udp.remotePort();
                    lastPeerRxMs = nowMs;
                    acceptedPeer = true;
                }
                sendToChassis(frame);
            }
        }
    }
}

void serviceUart(uint32_t nowMs) {
    size_t processed = 0;
    while (chassisUart.available() > 0 && processed < kMaxUartBytesPerLoop) {
        const int value = chassisUart.read();
        if (value < 0) {
            break;
        }
        ++processed;
        WireProtocol::Frame frame{};
        if (uartParser.feed(static_cast<uint8_t>(value), frame)) {
            sendToPeer(frame, nowMs);
        }
    }
}

void printStats(uint32_t nowMs) {
    if (static_cast<uint32_t>(nowMs - lastStatsMs) < kStatsPeriodMs) {
        return;
    }
    lastStatsMs = nowMs;
    const auto& wifiStats = udpParser.stats();
    const auto& chassisStats = uartParser.stats();
    const float chipTemperatureC = sampleChipTemperatureC();
    Serial.printf(
        "bridge ap=%u cfg=%u udp=%u ip=%s ch=%d sta=%u peer=%u "
        "ap_disc=%lu ap_restart=%lu "
        "udp_rx=%lu udp_ok=%lu udp_crc=%lu "
        "uart_tx=%lu bridge_tx=%lu uart_ok=%lu uart_crc=%lu "
        "udp_tx=%lu drop=%lu ms=%lu temp=%.1f tempMax=%.1f tempErr=%lu\n",
        apStartOk ? 1U : 0U, apConfigOk ? 1U : 0U,
        udpStartOk ? 1U : 0U, WiFi.softAPIP().toString().c_str(),
        WiFi.channel(), WiFi.softAPgetStationNum(),
        peerOnline(nowMs) ? 1U : 0U,
        static_cast<unsigned long>(apDisconnectEvents),
        static_cast<unsigned long>(apRestartCount),
        static_cast<unsigned long>(stats.udpDatagrams),
        static_cast<unsigned long>(wifiStats.validFrames),
        static_cast<unsigned long>(wifiStats.crcErrors),
        static_cast<unsigned long>(stats.udpToUart),
        static_cast<unsigned long>(stats.bridgeToUart),
        static_cast<unsigned long>(chassisStats.validFrames),
        static_cast<unsigned long>(chassisStats.crcErrors),
        static_cast<unsigned long>(stats.uartToUdp),
        static_cast<unsigned long>(stats.udpOversize + stats.udpSendErrors +
                                   stats.uartShortWrites),
        static_cast<unsigned long>(nowMs), chipTemperatureC,
        maxChipTemperatureC,
        static_cast<unsigned long>(temperatureReadErrors));
}

}  // namespace

void setup() {
    Serial.begin(115200);
    chassisUart.setRxBufferSize(256);
    chassisUart.begin(kChassisBaud, SERIAL_8N1, kChassisRxPin, kChassisTxPin);

    WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
    startAccessPoint();
    const float chipTemperatureC = sampleChipTemperatureC();

    Serial.printf(
        "bridge ready ssid=%s ap=%u cfg=%u ip=%s ch=%d udp=%u "
        "uart=%lu rx=%d tx=%d temp=%.1f\n",
        kAccessPointSsid, apStartOk ? 1U : 0U,
        apConfigOk ? 1U : 0U, WiFi.softAPIP().toString().c_str(),
        WiFi.channel(), udpStartOk ? kUdpListenPort : 0U,
        static_cast<unsigned long>(kChassisBaud), kChassisRxPin,
        kChassisTxPin, chipTemperatureC);
}

void loop() {
    const uint32_t nowMs = millis();
    serviceAccessPointRecovery(nowMs);
    serviceUdp(nowMs);
    serviceUart(nowMs);
    sendBridgePresence(nowMs);
    printStats(nowMs);
    delay(1);
}
