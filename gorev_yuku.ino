#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <Wire.h>


Adafruit_BME280 bme; 
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;

float pitch = 0, roll = 0, yaw = 0;
DateTime lastTime;
float dt;
float previousAltitude = 0.0;  // Bir önceki yükseklik
float descentRate = 0.0;       // İniş hızı

#define SEALEVELPRESSURE_HPA (1013.25)  // Deniz seviyesinde standart atmosfer basıncı (hPa)

// Verilerin taşınacağı yapı
typedef struct struct_message {
   float yükseklik1;  // Görev yükünden alınacak yükseklik1
  float yükseklik2;  // BMP085 sensöründen gelen yükseklik
  float basınç1;     // Görev yükünden alınacak basınç1
  float basınç2;     // BMP085 sensöründen gelen basınç
  int paketno;       // Paket numarası
  float sıcaklık;    // Görev yükünden alınacak sıcaklık
  float irtifafarkı; 
  float inişhızı;
  float pilgerilimi;
  float pitch;
  float roll;
  float yaw;
  char rhrh[10];     // Karakter dizisi
  float iotdata;     // Yer istasyonundan alınacak sıcaklık (DHT11)
  int takımno;
  int uydustatüsü;
  int hatakodu;
  char göndermesaati[25];
} struct_message;

struct_message mySensorData;  // Görev yükündeki veriler
struct_message incomingData;  // Taşıyıcıdan gelen veriler

// Yer istasyonuna ait MAC adresi (yer istasyonuna verileri yollayacağız)
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x8A, 0x75, 0xC4};  // Yer istasyonunun MAC adresi buraya

// Taşıyıcıdan gelen verileri aldığımız callback fonksiyonu
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataRaw, int len) {
  // Gelen verinin boyutunu kontrol et
  if (len == sizeof(incomingData)) {
    // Gelen veriyi struct_message yapısına kopyala
    memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

    // Gelen verileri seri port üzerinden yazdır
    Serial.printf("Taşıyıcıdan gelen veriler: basınç2 = %.2f Pa, yükseklik2 = %.2f m\n, iotdata = %.2f m\n",
                  incomingData.basınç2, incomingData.yükseklik2, incomingData.iotdata);
  } else {
    Serial.println("Veri boyutu uyuşmazlığı!");
  }
}

// Görev yükünden verileri yer istasyonuna göndermek için callback fonksiyonu
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Veri gönderimi: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Başarılı" : "Başarısız");
}

void setup() {
  // Seri haberleşmeyi başlat
  Serial.begin(115200);

  // Wi-Fi modülünü başlat
  WiFi.mode(WIFI_STA);

  // BME280 sensörünü başlat
  if (!bme.begin(0x76)) {
    Serial.println("BME280 sensörü bulunamadı, bağlantıları kontrol edin!");
    while (1);
  }

  // ESP-NOW'u başlat
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW başlatılamadı");
    return;
  }

    if (!mpu.begin(0x69)) {
    Serial.println("MPU6050 bulunamadı!");
    while (1);
  }
  
  // RTC başlatılıyor
  if (!rtc.begin()) {
    Serial.println("DS3231 bulunamadı!");
    while (1);
  }
  lastTime = rtc.now();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Peer ekle (yer istasyonu için)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));  // Yapıyı sıfırla
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // Yer istasyonunun MAC adresi
  peerInfo.channel = 0;  // Varsayılan kanal
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Yer istasyonu peer eklenemedi");
    return;
  }

  // Callback fonksiyonlarını ayarla
  esp_now_register_recv_cb(OnDataRecv);    // Taşıyıcıdan veri almak için
  esp_now_register_send_cb(OnDataSent);    // Yer istasyonuna veri göndermek için

  Serial.println("Görev yükü hazır.");
}

void loop() {

  DateTime now = rtc.now();
  
  // Zaman farkı hesaplama (saniye cinsinden)
  dt = now.secondstime() - lastTime.secondstime();
  lastTime = now;

  // MPU6050 sensöründen veri okuma
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Pitch ve roll hesaplama
  float accelPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;
  
  // Gyroscope verilerini kullanarak açıları hesaplama
  pitch += g.gyro.x * dt;
  roll += g.gyro.y * dt;
  yaw += g.gyro.z * dt;

  // Komplementer filtre ile ivmeölçer ve gyro verilerini birleştirme
  pitch = 0.98 * pitch + 0.02 * accelPitch;
  roll = 0.98 * roll + 0.02 * accelRoll;

  char dateTimeStr[25];  // Tarih ve saat için yeterli uzunlukta bir char dizisi
  snprintf(dateTimeStr, sizeof(dateTimeStr), "%04d/%02d/%02d %02d:%02d:%02d", 
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  // Tarih ve saat verisini dateTimeStr'den göndermesaati'ne kopyala
  strncpy(mySensorData.göndermesaati, dateTimeStr, sizeof(mySensorData.göndermesaati));
  mySensorData.göndermesaati[sizeof(mySensorData.göndermesaati) - 1] = '\0';  // Sonlandırıcı karakter

  // BME280 sensöründen verileri oku (görev yükünün kendi sensör verileri)
  mySensorData.basınç1 = bme.readPressure();
  mySensorData.yükseklik1 = bme.readAltitude(SEALEVELPRESSURE_HPA);
  mySensorData.sıcaklık = bme.readTemperature();
  mySensorData.takımno = 12345;
  mySensorData.pitch = pitch;
  mySensorData.roll = roll;
  mySensorData.yaw = yaw;

  // Taşıyıcıdan gelen BMP085 sensör verilerini al ve birleştir
  mySensorData.basınç2 = incomingData.basınç2;
  mySensorData.yükseklik2 = incomingData.yükseklik2;
  mySensorData.iotdata = incomingData.iotdata;
  mySensorData.irtifafarkı = abs(mySensorData.yükseklik1 - mySensorData.yükseklik2);

  // İniş hızını hesapla (m/s)
  if (dt > 0) {  // Zaman farkı sıfırdan büyük olmalı
    descentRate = (previousAltitude - mySensorData.yükseklik1) / dt;  // İniş hızı hesaplanır
  }
  mySensorData.inişhızı = descentRate;
  
  // Yüksekliği güncelle
  previousAltitude = mySensorData.yükseklik1;

  // Paket numarasını artır
  mySensorData.paketno++;

  // Tüm verileri seri port üzerinden yazdır
  Serial.printf("%d, %d, %d, %s, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %s, %.2f, %d \n", 
                  mySensorData.paketno, mySensorData.uydustatüsü, mySensorData.hatakodu, mySensorData.göndermesaati,  
                  mySensorData.basınç1, mySensorData.basınç2, mySensorData.yükseklik1, mySensorData.yükseklik2, 
                  mySensorData.irtifafarkı, mySensorData.inişhızı, mySensorData.sıcaklık, mySensorData.pilgerilimi, 
                  mySensorData.pitch, mySensorData.roll, mySensorData.yaw, mySensorData.rhrh, mySensorData.iotdata, mySensorData.takımno);

  // Tüm verileri yer istasyonuna gönder
  esp_now_send(broadcastAddress, (uint8_t *)&mySensorData, sizeof(mySensorData));

  // 2 saniye bekle
  delay(1000);
}
