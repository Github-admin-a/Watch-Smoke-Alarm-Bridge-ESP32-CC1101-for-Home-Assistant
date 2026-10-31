# Watch Smoke Alarm Bridge

**ESP32 + CC1101 bridge that listens to Watch brand RF-interconnected smoke alarms on 433 MHz and reports them to Home Assistant over MQTT.**

The bridge receives the radio messages the smoke detectors and the wall controller send to each other. When any alarm goes off it turns on a smoke sensor in Home Assistant, logs the event and shows which detector sent it. It also has a web interface for setup, an event log and some radio tools for working out the signal.

> ⚠️ **This is a hobby project, not a certified safety device.** It does not replace your smoke alarms or a monitored fire service. It only *listens*: if the bridge, Wi-Fi, MQTT or Home Assistant is down you won't get a notification. Your alarms keep working as normal either way.

---

## Features

- **Receives the alarm network** – 2-FSK, ~100 kbaud, sync word `2F33`, 8-byte frames. It's tuned to 433.30 MHz with a 325 kHz window, so it picks up both the wall controller (~433.25 MHz) and the detectors (~433.35 MHz). Automatic frequency correction is on.
- **Classifies frames** as *Alarm*, *Silence* or *Unknown* using the system ID and command code. A small number of bit errors is corrected by matching against known frames.
  - The wall panel's TEST button sends the same frame as a detector that's going off, so the bridge can't tell them apart. Every alarm/test frame is reported as **Alarm**.
- **Shows which detector triggered** – every transmitter's crystal is a little off-frequency, and the bridge records that offset for each event. The **Transmitters** page has a *Learn* button: press it, set off an alarm, and give it a name. After that, events show the name, e.g. "Hallway".
- **Event log** with time, type, device and RSSI, stored in flash (LittleFS) so it survives a reboot. The main page shows one row per button press.
- **Home Assistant MQTT discovery** – no YAML needed. It creates:
  - `binary_sensor` – Smoke alarm (device class `smoke`), turns off automatically after a set time
  - `sensor` – Last event, Last event time, Triggered by, Alarm RF signal (RSSI), IP address
  - `sensor` – Event log (last 20 events as attributes, ready for a dashboard table)
  - `event` – Smoke alarm events (history + automation trigger)
  - `button` – Clear event log
- **Diagnostics page** – burst log (each transmission with duration, copies, RSSI, bit fixes), unknown frames, radio stats.
- **Scanner page** – sweeps a frequency range to find where signals appear, plus a raw capture mode that works out the bit rate, preamble and frame of any FSK signal. Handy if your system uses different settings.
- **Optional TEST / SILENCE transmit buttons** – **off by default**. Turn them on in the setup page if you want to test.
- Wi-Fi setup through a captive portal (WiFiManager), mDNS (`http://smokealarm.local/`), settings saved to flash.

---

## Hardware

- ESP32-C3 Super Mini **or** a classic ESP32 dev board (WROOM-32)
- CC1101 module, **433 MHz version** (check the antenna/filter – many cheap modules are 868/915 MHz)
- A 433 MHz antenna (a 17.3 cm piece of wire works)

### Wiring

The sketch picks the pins automatically from the board selected in the Arduino IDE.

| CC1101 | ESP32-C3 Super Mini | ESP32 (WROOM-32) |
|--------|---------------------|------------------|
| VCC    | 3V3                 | 3V3              |
| GND    | GND                 | GND              |
| SCK    | GPIO4               | GPIO14           |
| MISO   | GPIO5               | GPIO25           |
| MOSI   | GPIO6               | GPIO26           |
| CSN    | GPIO7               | GPIO13           |
| GDO0   | GPIO3 *             | GPIO32 *         |

\* GDO0 is only needed for the Scanner page's raw capture. Normal alarm monitoring works without it.

**The CC1101 is a 3.3 V part – don't connect it to 5 V.**

---

## Software

### Arduino IDE

- ESP32 board package (Espressif)
- Board:
  - ESP32-C3 Super Mini → **ESP32C3 Dev Module**, *USB CDC On Boot: Enabled*
  - WROOM-32 → **ESP32 Dev Module**

### Libraries (Library Manager)

- **WiFiManager** by tzapu
- **PubSubClient** by Nick O'Leary
- **ArduinoJson** v7 by Benoit Blanchon

The CC1101 is driven directly over SPI, so no CC1101 library is needed.

---

## Getting started

1. Open `SmokeAlarm_Bridge_v3_3.ino`, select your board and upload.
2. On first boot the ESP32 starts a Wi-Fi access point called **`SmokeAlarmBridge-Setup`**. Join it and enter your Wi-Fi details.
3. Browse to **http://smokealarm.local/** (or the IP address shown on the serial monitor at 115200 baud).
4. Open the **Wi-Fi / MQTT** page and enter your MQTT broker, username and password. Set your time zone (POSIX format, e.g. `UTC0`, or `AEST-10AEDT,M10.1.0,M4.1.0/3` for Sydney/Melbourne. The default is UTC).
5. The device appears in Home Assistant under **Settings → Devices & services → MQTT**.

No passwords or network details are stored in the code – everything is entered through the web pages and saved on the device.

### Setting up your own alarm system

The System ID ships as a placeholder (`0000000000`), so you need to enter your own before alarms are recognised. The Silence and Alarm/Test codes are the ones seen on the author's system and may differ on yours. To find them:

1. Press **Silence** and **Test** on the wall controller.
2. Look at the **Diagnostics** page – frames from your system show up as *Unknown* along with their hex.
3. On the **Alarm setup** page, enter the first 5 bytes as the **System ID** and the remaining 3 bytes as the **Silence** and **Alarm/Test** codes.
4. If nothing shows up at all, use the **Scanner** page to find the frequency your alarms use.

### Naming your detectors

1. Go to the **Transmitters** page and press **Learn**.
2. Set off one detector (its test button).
3. The bridge saves that detector's frequency offset – give it a name.
4. Repeat for each detector.

Detectors with very similar crystals can end up with almost the same offset, so not every installation can tell every detector apart. Adjust the offset tolerance on the setup page if needed.

---

## Home Assistant

### MQTT topics

The base topic is `smokealarm` by default (it can be changed).

| Topic | Content |
|-------|---------|
| `smokealarm/status` | `online` / `offline` (LWT) |
| `smokealarm/state` | `ON` / `OFF` |
| `smokealarm/last_event` | Last event type |
| `smokealarm/last_event_time` | ISO timestamp |
| `smokealarm/last_device` | Name of the detector that triggered |
| `smokealarm/rf_event` | Event entity payload (`alarm`, `silence`, `test`, `unknown`) |
| `smokealarm/log` | Last 20 events (JSON) |
| `smokealarm/rssi` | RSSI of the last alarm frame |
| `smokealarm/ip` | Bridge IP address |
| `smokealarm/clear_log` | Command topic – clears the event log |

### Example automation

```yaml
alias: Smoke alarm notification
triggers:
  - trigger: state
    entity_id: binary_sensor.smoke_alarm_bridge_smoke_alarm
    to: "on"
actions:
  - action: notify.notify
    data:
      title: "🔥 Smoke alarm"
      message: >
        Smoke alarm triggered by
        {{ states('sensor.smoke_alarm_bridge_triggered_by') }}
```

Check the entity IDs in your own Home Assistant – they depend on the device name you set.

---

## Notes

- The **Device ID** setting is the Home Assistant identity. Keep it the same if you swap boards and HA will keep the same entities.
- The Scanner and raw capture **pause alarm monitoring** while they run. Both stop by themselves after 10 minutes.
- Only use the transmit buttons on your own alarm system, and remember your neighbours may have the same brand.
- Tested with Watch brand RF smoke alarms and wall controller. Other brands that use the same radio module may work but haven't been tested.

---

## Licence

MIT – see `LICENSE`. Use at your own risk.
