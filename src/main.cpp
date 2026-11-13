#include <Arduino.h>
#include <M5Unified.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

// M5Stack Core2からKRS-3204へ位置指令を送り、応答を画面に表示する。
// 起動時には動かさず、本体ボタンまたはUSBシリアルからの操作時だけ送信する。
// 配線はREADME.mdを参照。RXとオープンドレインTXをレベル変換回路の
// 3.3V側で共有する。5VのICS信号をESP32のGPIOに直接接続しないこと。
// See README.md: RX and open-drain TX share the 3.3 V side of a MOSFET
// translator. Never connect the 5 V ICS signal directly to an ESP32 GPIO.
namespace Config {
// 通信先のID・通信速度は、サーボに保存されている設定と合わせる。
constexpr int rxPin = 13;
constexpr int txPin = 14;
//constexpr int rxPin = 33;
//constexpr int txPin = 32;
constexpr uint8_t servoId = 0;
constexpr uint32_t baud = 115200;
constexpr uint32_t timeoutMs = 30;
// 位置は角度（度）ではなくICSの位置値。初期確認用に7000～8000へ制限する。
constexpr uint16_t center = 7500;
constexpr uint16_t minPosition = 7000;  // Limited travel for initial setup.
constexpr uint16_t maxPosition = 8000;
constexpr int step = 100;
}

// IDと位置制限に矛盾があれば、コンパイル時にエラーにする。
static_assert(Config::servoId <= 31, "ICS ID must be 0..31");
static_assert(Config::minPosition >= 3500 && Config::maxPosition <= 11500 &&
              Config::minPosition <= Config::center &&
              Config::center <= Config::maxPosition, "Invalid position limits");

HardwareSerial ics(2);
// UART2をサーボ通信用に使用。SerialはUSB経由の操作・ログ出力用。
// targetPositionは次の増減操作の基準値で、起動時にはまだ送信されていない。
uint16_t targetPosition = Config::center;
// -1は有効な応答がない状態。受信位置は目標到達を保証する値ではない。
int actualPosition = -1;
const char* status = "IDLE (no command sent)";
bool busReady = false;

// Require both TX echo and servo reply. Echo alone is NOT a servo response.
// position=0はトルク解除（FREE）、それ以外は位置指令。
// 送信エコーとサーボ応答の両方を検証し、成功時だけactualへ受信位置を返す。
bool exchangePosition(uint16_t position, uint16_t& actual) {
  if (!busReady) {
    status = "UART init failed";
    return false;
  }
  if (position != 0 &&
      (position < Config::minPosition || position > Config::maxPosition)) {
    status = "Position out of range";
    return false;
  }
  // 前回の受信データを捨て、送信前に信号線が待機状態のHIGHか確認する。
  while (ics.available()) ics.read();
  if (digitalRead(Config::rxPin) == LOW) {
    status = "Bus stuck LOW";
    return false;
  }
  // 先頭は位置指令を表す0x80とID。位置値は上位・下位各7ビットに分ける。
  const uint8_t command[] = {
      static_cast<uint8_t>(0x80 | Config::servoId),
      static_cast<uint8_t>((position >> 7) & 0x7f),
      static_cast<uint8_t>(position & 0x7f)};
  if (ics.write(command, sizeof(command)) != sizeof(command)) {
    status = "UART write failed";
    return false;
  }
  // Open-drain HIGH releases the wire during stop bits and the entire reply. TXに関する説明
  // TXはHIGH時に信号線を開放するため、サーボが同じ線へ応答できる。
  // 送信完了だけを待ち、既にRXへ届いたエコーや応答は消さずに残す。
  ics.flush(true);  // TX only: preserve echo and early reply in RX.
  // RX/TXを共有しているので、自分の送信エコー3バイト＋応答3バイトを待つ。
  uint8_t received[6] = {};
  size_t count = 0;
  const uint32_t started = millis();
  while (count < sizeof(received) && millis() - started < Config::timeoutMs) {
    if (ics.available()) received[count++] = static_cast<uint8_t>(ics.read());
    else delay(1);
  }
  if (count < 3) {
    status = "No echo: check RX/TX";
    return false;
  }
  // 最初の3バイトが送信内容と一致しても、サーボからの応答とは限らない。
  for (size_t i = 0; i < sizeof(command); ++i) {
    if (received[i] != command[i]) {
      status = "Echo mismatch / collision";
      return false;
    }
  }
  if (count != sizeof(received)) {
    status = "No reply: power/ID/rate";
    return false;
  }
  Serial.printf("Echo: %02X %02X %02X | Reply: %02X %02X %02X\n",
                received[0], received[1], received[2],
                received[3], received[4], received[5]);
  // Position response clears the command MSB; payload bytes are 7-bit.
  // 応答の先頭が対象IDで、位置データの各最上位ビットが0であることを確認する。
  if (received[3] != (0x80 | Config::servoId) || (received[4] & 0x80) ||
      (received[5] & 0x80)) {
    status = "Invalid ICS reply";
    return false;
  }
  // 2つの7ビットデータを結合し、応答時点の位置値へ戻す。
  actual = (static_cast<uint16_t>(received[4]) << 7) | received[5];
  return true;
}

// 目標値・直近の応答位置・通信結果・操作方法を画面全体に描き直す。
void drawScreen() {
  auto& lcd = M5.Display;
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(8, 8);
  lcd.println("KRS-3204 ICS / Core2");
  lcd.printf("ID:%u  %lu 8E1\n", Config::servoId,
             static_cast<unsigned long>(Config::baud));
  lcd.printf("Target: %u\n", targetPosition);
  if (actualPosition >= 0) lcd.printf("Reply:  %d\n", actualPosition);
  else lcd.println("Reply:  ----");
  lcd.setTextSize(1);
  lcd.setCursor(8, 116);
  lcd.println(status);
  lcd.println("Reply is sampled position, not arrival confirmation.");
  lcd.println("USB: j=-100 k=+100 c=center f=free");
  lcd.println("FREE releases torque and reads position.");
  lcd.setTextSize(2);
  lcd.setCursor(8, 210);
  lcd.print("-100    FREE    +100");
}

// 通信結果を管理し、USBログと画面を更新する。
void sendPosition(uint16_t position) {
  uint16_t actual = 0;
  if (exchangePosition(position, actual)) {
    actualPosition = actual;
    if (position == 0) {
      status = "FREE confirmed";
      // FREE後の増減操作は受信位置を基準にする。ただし設定範囲内に制限する。
      targetPosition = constrain(actual, Config::minPosition, Config::maxPosition);
    } else {
      targetPosition = position;
      status = "Position command acknowledged";
    }
  } else {
    // A lost reply does not mean the command failed to reach the servo.
    // Do not retry motion automatically or claim torque has been released.
    // 応答がなくても指令は届いている可能性があるため、自動再送はしない。
    // 表示上の応答を無効にするだけで、停止やトルク解除を保証するものではない。
    actualPosition = -1;
  }
  Serial.printf("%s | requested=%u reply=%d\n", status, position, actualPosition);
  drawScreen();
}

// 目標値に増減量を加え、上下限を超えないようconstrainで制限して送信する。
// 減算時の符号なし整数の桁あふれを避けるため、計算前にintへ変換する。
void stepPosition(int delta) {
  sendPosition(constrain(static_cast<int>(targetPosition) + delta,
                         Config::minPosition, Config::maxPosition));
}

// 起動時に一度だけ実行する初期化処理。
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Display.setRotation(1);

  // サーボ用UARTは8データビット・偶数パリティ・1ストップビット（8E1）。
  ics.begin(Config::baud, SERIAL_8E1, Config::rxPin, Config::txPin);
  // gpio_set_direction() disconnects UART output routing in ESP-IDF 4.4.7.
  // UARTのピン割り当てを保ち、オープンドレイン設定ビットだけを変更する。
  busReady = static_cast<bool>(ics);
  if (busReady) {
    gpio_ll_od_enable(&GPIO, static_cast<gpio_num_t>(Config::txPin));
  }
  if (!busReady) status = "UART init failed";
  // ICS3.5 selects serial mode after >=500 ms HIGH at servo power-on.
  // Power Core2 first, then the servo; wait >=1 s before operating it.
  // Core2を先に起動し、サーボ電源投入後も操作まで1秒以上待つこと。
  // この待機処理では位置指令を送らないため、起動だけでは中央へ移動しない。
  delay(1000);
  Serial.println("ICS ready. j/k: step, c: center, f: free. No startup motion.");
  drawScreen();
  sendPosition(0);
}

// ボタンとUSB入力を繰り返し確認する。押し続けではなく押した瞬間に反応する。
void loop() {
  M5.update();
  // B: FREE、A: 目標値を減らす、C: 目標値を増やす。同時押しではBを優先する。
  if (M5.BtnB.wasPressed()) sendPosition(0);
  else if (M5.BtnA.wasPressed()) stepPosition(-Config::step);
  else if (M5.BtnC.wasPressed()) stepPosition(Config::step);
  // USBから1文字ずつ読む。j/k: 増減、c: 中央、f: FREE。改行などは無視する。
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'j': stepPosition(-Config::step); break;
      case 'k': stepPosition(Config::step); break;
      case 'c': sendPosition(Config::center); break;
      case 'f': sendPosition(0); break;
      default: break;
    }
  }
  delay(5);
}
