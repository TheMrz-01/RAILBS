# Raylı Araç ESP32 Projesi

Bu repo, raylı araç yarışması için geliştirilen ESP32 tabanlı otonom araç yazılımını ve bilgisayarda çalışan web kontrol/log panelini içerir.

Araç tarafında ESP32 şunları yapar:

- İki adet RS550 DC motoru BTS7960B motor sürücüleri ile kontrol eder.
- KY-024 manyetik sensörler ile ray üzerindeki marker/mıknatıs geçişlerini sayar.
- HC-SR04 ultrasonik sensör ile tünel algılaması yapar.
- Tünel içinde belirlenen markerda durup yaklaşık 5 saniye bekler.
- Bitişe yaklaşınca yavaşlar ve finish markerında durur.
- Kendi WiFi ağı üzerinden durum, test modu ve ayar arayüzü sunar.

Bilgisayar tarafındaki web paneli şunları yapar:

- ESP32'den canlı veri okur.
- Grafikler gösterir.
- Sürüş loglarını bilgisayara kaydeder.
- CSV/JSON dosyaları indirilebilir hale getirir.
- ESP32 ayarlarını web üzerinden değiştirmeyi sağlar.

## Klasör Yapısı

```text
Esp32-Testy-Test/
  Embedded/          ESP32 firmware kodu
    src/main.cpp
    platformio.ini

  Web/               Bilgisayarda çalışan Bun web paneli
    src/server.ts
    public/index.html
    public/app.js
    public/style.css

  README.md          Bu rehber
  rapor.txt          Rapor için hazırlanmış açıklama metni
```

## Gerekenler

Bilgisayarda şunlar kurulu olmalı:

- Visual Studio Code
- PlatformIO
- Bun
- Git
- ESP32 USB sürücüsü

Gerekli donanım:

- ESP32 geliştirme kartı
- 2 adet BTS7960B motor sürücü
- 2 adet RS550 DC motor
- 2 adet KY-024 manyetik sensör
- 1 adet HC-SR04 ultrasonik sensör
- Uygun batarya
- Sigorta ve acil kesme anahtarı
- ESP32 için buck converter veya güvenli 5V besleme

## 1. Repoyu İndirme

Git kuruluysa terminalden:

```bash
git clone <repo-linki>
```

Sonra klasöre gir:

```bash
cd Esp32-Testy-Test
```

Git kullanmak istemiyorsan GitHub üzerinden `Code > Download ZIP` seçeneği ile indirip ZIP dosyasını çıkarabilirsin.

## 2. Visual Studio Code Kurulumu

Visual Studio Code indir:

```text
https://code.visualstudio.com/
```

Kurulumdan sonra proje klasörünü VS Code ile aç:

```text
File > Open Folder > Esp32-Testy-Test
```

## 3. PlatformIO Kurulumu

PlatformIO, ESP32 kodunu derlemek ve karta yüklemek için kullanılır.

VS Code içinde:

```text
Extensions > PlatformIO IDE > Install
```

Kurulumdan sonra VS Code'u kapatıp tekrar açmak iyi olur.

PlatformIO terminalden çalışıyor mu kontrol et:

```bash
pio --version
```

Eğer `pio` bulunamazsa macOS/Linux için genelde şu komut çalışır:

```bash
~/.platformio/penv/bin/pio --version
```

Bu projede komut örneklerinde iki yöntem de gösterilmiştir.

## 4. ESP32 USB Portunu Bulma

ESP32'yi USB ile bilgisayara bağla.

macOS için:

```bash
ls /dev/cu.*
```

Örnek port:

```text
/dev/cu.usbserial-0001
```

Windows için port genelde şöyle görünür:

```text
COM3
COM4
COM5
```

VS Code PlatformIO panelinden de port görülebilir.

## 5. PlatformIO Port Ayarı

ESP32 firmware ayar dosyası burada:

```text
Embedded/platformio.ini
```

İçinde şu satırlar vardır:

```ini
upload_port = /dev/cu.usbserial-0001
monitor_port = /dev/cu.usbserial-0001
monitor_speed=115200
```

Kendi portun farklıysa değiştir.

macOS örneği:

```ini
upload_port = /dev/cu.usbserial-XXXX
monitor_port = /dev/cu.usbserial-XXXX
```

Windows örneği:

```ini
upload_port = COM3
monitor_port = COM3
```

Önemli: `monitor_speed` baud rate içindir. `monitor_port` içine `115200` yazılmaz.

Doğru:

```ini
monitor_speed = 115200
```

Yanlış:

```ini
monitor_port = 115200
```

## 6. ESP32 Kodunu Derleme

Proje kök klasöründeyken:

```bash
pio run -d Embedded
```

Eğer `pio` bulunamazsa:

```bash
~/.platformio/penv/bin/pio run -d Embedded
```

Başarılı derleme sonunda `SUCCESS` görmelisin.

## 7. ESP32'ye Kod Yükleme

Proje kök klasöründeyken:

```bash
pio run -d Embedded -t upload
```

Eğer `pio` bulunamazsa:

```bash
~/.platformio/penv/bin/pio run -d Embedded -t upload
```

Yükleme sırasında ESP32 boot moduna geçmezse kart üzerindeki `BOOT` tuşuna basılı tutmak gerekebilir.

Genel yöntem:

1. Upload komutunu çalıştır.
2. Terminal `Connecting...` yazınca ESP32 üzerindeki `BOOT` tuşuna basılı tut.
3. Yükleme başlayınca bırak.

## 8. Serial Monitor Açma

ESP32'nin seri çıktısını görmek için:

```bash
pio device monitor -d Embedded
```

Eğer `pio` bulunamazsa:

```bash
~/.platformio/penv/bin/pio device monitor -d Embedded
```

Baud rate:

```text
115200
```

Seri ekranda anlamsız karakterler görürsen genelde baud rate yanlıştır. `platformio.ini` içinde şunun olduğundan emin ol:

```ini
monitor_speed = 115200
```

## 9. ESP32 Yerel Web Paneline Erişim

Firmware yüklendikten sonra ESP32 kendi WiFi ağını açar.

WiFi bilgileri:

```text
SSID: KralVonMobil
Şifre: MustiSuckz
```

Bilgisayardan veya telefondan bu WiFi ağına bağlan.

Tarayıcıda aç:

```text
http://192.168.4.1
```

Bu panelden şunları yapabilirsin:

- Görevi başlatma
- Acil durdurma
- Sayaçları sıfırlama
- Test moduna girme
- Motorları tek tek test etme
- PWM değerlerini değiştirme
- Marker ve parkur ayarlarını değiştirme

## 10. Test Modu Kullanımı

Test modu, motorları otonom görev başlatmadan manuel denemek içindir.

Önemli güvenlik uyarısı:

```text
Test modunu sadece araç havadayken veya güvenli şekilde sabitlenmişken kullan.
```

ESP32 panelinde:

1. `Test Moduna Gir` butonuna bas.
2. PWM değerini düşük seç. Örneğin `60` veya `80`.
3. `İleri`, `Geri`, `Sol Motor`, `Sağ Motor` butonlarıyla motorları dene.
4. İş bitince `Boşa Al` veya `Frenle` kullan.
5. Sonra `Test Modundan Çık` butonuna bas.

Test modu aktifken otonom algoritma motor PWM değerlerini ezmez.

## 11. Bun Kurulumu

Bun, bilgisayardaki web dashboard'u çalıştırmak için kullanılır.

Bun kurulum sayfası:

```text
https://bun.sh/
```

macOS/Linux için genelde:

```bash
curl -fsSL https://bun.sh/install | bash
```

Kurulumdan sonra terminali kapatıp aç.

Kontrol et:

```bash
bun --version
```

## 12. Web Dashboard'u Çalıştırma

Proje kök klasöründeyken:

```bash
bun run --cwd Web dev
```

Tarayıcıda aç:

```text
http://localhost:3000
```

Dashboard varsayılan olarak ESP32'ye şu adresten bağlanmaya çalışır:

```text
http://192.168.4.1
```

Bu yüzden bilgisayarın ESP32 WiFi ağına bağlı olmalıdır.

ESP32 bağlantısını test etmek için:

```bash
curl http://192.168.4.1/status
```

JSON veri dönüyorsa bağlantı çalışıyor demektir.

## 13. Web Dashboard Ne İşe Yarar?

Dashboard üzerinden şunları görebilirsin:

- Araç durumu
- Görev süresi
- Ön marker sayısı
- Arka marker sayısı
- Ultrasonik mesafe
- Tahmini hız
- Sol/sağ PWM
- Canlı grafikler
- Kaydedilen sürüşler

Dashboard ayrıca sürüş loglarını bilgisayara kaydeder.

Kayıt klasörü:

```text
Web/runs/
```

Her sürüş için şunlar oluşur:

```text
telemetry.csv
events.csv
summary.json
```

Bu dosyalar web panelinden indirilebilir.

## 14. ESP32 Ayarları

Hem ESP32 yerel panelinden hem de bilgisayar dashboard'undan bazı ayarlar değiştirilebilir.

Önemli ayarlar:

```text
Seyir PWM
Yaklaşma PWM
Bitiş PWM
Fren PWM
Motor Trim
Marker Aralığı cm
Parkur Mesafesi cm
Tünel Yaklaşma Markerı
Tünelde Durma Markerı
Tünel Çıkış Markerı
Bitiş Yaklaşma Markerı
Bitiş Markerı
Tünel Durma ms
Tünel Mesafe Eşiği cm
Manyetik Debounce us
```

Varsayılan önemli değerler:

```text
Marker aralığı: 50 cm
Parkur mesafesi: 2000 cm
Tünel yaklaşma markerı: 17
Tünelde durma markerı: 19
Tünel çıkış markerı: 20
Bitiş yaklaşma markerı: 37
Bitiş markerı: 40
Tünel durma süresi: 5100 ms
```

Ayarlar ESP32'nin kalıcı hafızasına kaydedilir. Kart resetlense bile ayarlar korunur.

## 15. Normal Kullanım Sırası

Geliştirme/test için önerilen sıra:

1. ESP32'yi USB ile bağla.
2. Firmware'i yükle:

```bash
pio run -d Embedded -t upload
```

3. ESP32 WiFi ağına bağlan:

```text
KralVonMobil
```

4. ESP32 panelini aç:

```text
http://192.168.4.1
```

5. Gerekirse test modunda motorları dene.
6. Bilgisayar dashboard'unu başlat:

```bash
bun run --cwd Web dev
```

7. Dashboard'u aç:

```text
http://localhost:3000
```

8. Araç ayarlarını kontrol et.
9. Görevi başlat.
10. Sürüşten sonra logları indir.

## 16. Önemli Bağlantı Notları

Motor sürücü bağlantıları kodda şu şekildedir:

```cpp
LEFT_RPWM_PIN = 25
LEFT_LPWM_PIN = 26
RIGHT_RPWM_PIN = 18
RIGHT_LPWM_PIN = 5
```

Manyetik sensörler:

```cpp
FRONT_MAGNET_PIN = 34
REAR_MAGNET_PIN = 35
```

Ultrasonik sensör:

```cpp
ULTRASONIC_TRIG_PIN = 27
ULTRASONIC_ECHO_PIN = 14
```

HC-SR04 ECHO pini genelde 5V verir. ESP32 pinleri 5V toleranslı değildir.

Bu yüzden ECHO pini için voltaj bölücü kullan:

```text
HC-SR04 ECHO -> direnç bölücü -> ESP32 GPIO14
```

Örnek:

```text
ECHO -> 1k -> GPIO14
GPIO14 -> 2k -> GND
```

Tüm topraklar ortak olmalıdır:

```text
ESP32 GND
BTS7960B GND
Batarya eksi
```

Motorları ESP32 USB beslemesinden çalıştırma. Motorlar bataryadan, ESP32 ise güvenli 5V regülatörden beslenmelidir.

## 17. Sık Karşılaşılan Sorunlar

### `pio: command not found`

PlatformIO terminal PATH'e eklenmemiş olabilir.

Şunu kullan:

```bash
~/.platformio/penv/bin/pio run -d Embedded
```

### Serial monitörde garip karakterler çıkıyor

Baud rate yanlış olabilir.

Kontrol et:

```ini
monitor_speed = 115200
```

### `could not open port 115200` hatası

`platformio.ini` içinde yanlışlıkla şu yazılmış olabilir:

```ini
monitor_port = 115200
```

Bu yanlıştır. Doğrusu:

```ini
monitor_speed = 115200
```

### Dashboard ESP32'ye bağlanmıyor

Bilgisayarın ESP32 WiFi ağına bağlı olduğundan emin ol:

```text
KralVonMobil
```

Sonra test et:

```bash
curl http://192.168.4.1/status
```

### ESP32 upload olmuyor

Şunları kontrol et:

- USB kablosu data destekli mi?
- Doğru port seçildi mi?
- `BOOT` tuşuna basmak gerekiyor mu?
- ESP32 sürücüsü kurulu mu?

### Motor dönmüyor

Şunları kontrol et:

- BTS7960B `R_EN` ve `L_EN` pinleri aktif mi?
- Motor bataryası bağlı mı?
- ESP32 GND ve motor sürücü GND ortak mı?
- Test modunda düşük PWM ile denendi mi?
- Motor yönü tersse motor kabloları ters çevrilebilir.

### KY-024 marker saymıyor

Şunları kontrol et:

- KY-024 VCC 3.3V'a bağlı mı?
- D0 pini doğru ESP32 pinine bağlı mı?
- Potansiyometre ayarı doğru mu?
- Kodda interrupt tipi `FALLING`; sensör ters çalışıyorsa `RISING` gerekebilir.

## 18. Faydalı Komutlar

Kök klasörden firmware build:

```bash
pio run -d Embedded
```

Firmware upload:

```bash
pio run -d Embedded -t upload
```

Serial monitor:

```bash
pio device monitor -d Embedded
```

Bun dashboard başlat:

```bash
bun run --cwd Web dev
```

Bun dashboard normal başlat:

```bash
bun run --cwd Web start
```

Bun dashboard check:

```bash
bun run --cwd Web check
```

Farklı dashboard portu kullan:

```bash
PORT=3100 bun run --cwd Web dev
```

Farklı ESP32 IP kullan:

```bash
ESP32_URL=http://ESP32_IP_ADRESI bun run --cwd Web dev
```

## 19. Yarışma İçin Önemli Not

Araç otonom görev sırasında web paneline bağlı olmak zorunda değildir. ESP32 içindeki algoritma kendi başına görevi tamamlayacak şekilde çalışır.

Web paneli temel olarak şunlar içindir:

- Test
- Ayar
- Loglama
- Grafik izleme

Resmi görev sırasında yarışma kurallarına göre start komutundan sonra araca müdahale edilmemelidir.
