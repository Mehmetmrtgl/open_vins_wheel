# ZED Link Driver Kurulumu

Bu doküman, **ZED Link Mono GMSL2 Capture Card** kullanarak  
**NVIDIA Jetson Orin NX** üzerinde ZED GMSL2 kameraların çalışması için gerekli olan **ZED Link Driver** kurulum adımlarını açıklar.

---

## Ön Koşullar

Aşağıdaki donanım ve yazılımların hazır olduğundan emin olun:

- NVIDIA® Jetson™ Orin NX
- ZED Link Mono GMSL2 Capture Card
- GMSL2 kamera (ZED X serisi)
- Jetson üzerinde çalışan uygun **Jetson Linux (L4T)** sürümü
- İnternet bağlantısı

> **Not:** Donanım bağlantıları tamamlandıktan sonra bu kuruluma geçilmelidir.

---

## ZED Link Driver Nedir?

ZED Link Driver;

- GMSL2 deserializer kartını yapılandırır
- Jetson carrier board ile kameralar arasındaki iletişimi sağlar
- Kamera sırası, port yapılandırması ve donanım eşleşmelerini yönetir

Her **ZED Link kart modeli (MONO / DUO / QUAD)** için **farklı bir driver** vardır.

---

## ZED Link Driver İndirme

Driver, Stereolabs resmi sitesinden indirilmelidir:

- **ZED X Camera Drivers** (https://www.stereolabs.com/en-tr/developers/drivers) sayfasına gidin
- Jetson modelinize ve L4T sürümünüze uygun driver’ı seçin

> **Öneri:** Her zaman **en güncel sürümü** kullanın.

---

## ZED Link Driver Kurulumu

İndirdiğiniz `.deb` dosyasının bulunduğu dizine gidin ve aşağıdaki komutu çalıştırın:

```bash
sudo dpkg -i stereolabs-zedx_X.X.X-ZED-LINK-YYYY-L4TZZ.Z_arm64.deb
```

## Gerekli Bağımlılıklar

Eğer sisteminizde kurulu değilse aşağıdaki paketi yükleyin:

```bash
sudo apt install libqt5core5a
```

##  Sistemi Yeniden Başlatma

Driver kurulumundan sonra Jetson’ı yeniden başlatın:
```bash
sudo reboot
```

## Driver’ın Yüklendiğini Kontrol Etme

Sistem açıldıktan sonra aşağıdaki komut ile driver’ın yüklendiğini doğrulayın:

```bash
sudo dmesg | grep zedx
```
## Kamera Değişiklikleri ve Daemon Yeniden Başlatma

> GMSL2 kameralar hot-plug desteklemez.
Aşağıdaki durumlarda işlem gerekir:

 - Kamera sökme / takma

 - Kamera sırasını değiştirme

 - Farklı porta takma
## ZED daemon yeniden başlatma

```bash
sudo systemctl restart zed_x_daemon
```