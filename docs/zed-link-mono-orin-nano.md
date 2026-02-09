# ZED Link Mono on Orin Nano

Bu doküman, **ZED Link Mono GMSL2 capture card** kullanarak  
**Stereolabs GMSL2 kameraların**, **NVIDIA® Jetson™ Orin** üzerinde nasıl kurulacağını ve çalıştırılacağını açıklar.

---

## Gereksinimler

Kuruluma başlamadan önce aşağıdaki donanımların hazır olduğundan emin olun:

- 1x NVIDIA® Jetson™ Orin NX 
- 1x ZED Link Mono GMSL2 Capture Card
- 1x NVIDIA® güç adaptörü (DevKit ile birlikte gelir)
- 1x 22-pin 0.5mm pitch FPC kamera kablosu
- 12V – 19V (min. 2W) güç kaynağı  
  (5.5 mm dış / 2.5 mm iç çap – capture card için)
- GMSL2 kamera ve Fakra kablosu

## Donanım Kurulumu

>Kuruluma başlamadan önce **tüm güç bağlantılarının kesilmiş** olduğundan emin olun.

### 1. Capture Card Güç Bağlantısı

- Güç adaptörü kablosunu capture card üzerindeki **PWR J3** konektörüne bağlayın.

---

### 2. MIPI (CSI) Kabloları

- 22-pin MIPI kablosunu capture card üzerindeki porta bağlayın
- Kablonun yönünü doğru taktığınızdan emin olun

> **Uyarı:**  
Kablonun ters takılması capture card veya carrier board’a zarar verebilir.

---

### 3. Carrier Board Bağlantısı

- MIPI kablosunu Jetson Orin Nano üzerindeki 22-pin CSI konektörüne bağlayın
- Pin yönlerini doğru hizalayın

---

### 4. GMSL2 Kamera Bağlantısı

- Fakra kablosunun bir ucunu kamera üzerindeki GMSL2 girişine bağlayın diğer ucunu ise capture card üzerindeki GMSL2 girişine bağlayın

> **Not:**  
GMSL2 kameraların **boot öncesi takılı olması önerilir**.  
Eğer sonradan takıldıysa, `zed_x_daemon` yeniden başlatılmalıdır.

---

### 5. Güç Verme

- Capture card’a 12V–19V güç verin (PoC üzerinden kamera beslenir)
- Ardından Jetson DevKit’i güç kaynağına bağlayın
- Sistem açılana kadar bekleyin

---

## Yazılım Kurulumu

Donanım kurulumu tamamlandıktan sonra aşağıdaki yazılım adımlarını izleyin.

### 1. ZED Link Driver Kurulumu

ZED Link Mono capture card’ın çalışması için **özel driver** kurulmalıdır.

Ayrıntılı adımlar için:  
[ZED Link Driver Kurulumu](install-zed-link-driver.md)

---

### 2. ZED SDK Kurulumu (Linux)

Kameraların test edilmesi ve API’lerin kullanılabilmesi için **ZED SDK** gereklidir.

Ayrıntılı adımlar için:  
[Linux Üzerinde ZED SDK Kurulumu](install-zed-sdk-linux.md)

---

## Kamera Yapılandırma Notları

- GMSL2 kameralar **USB kameralar gibi esnek değildir**
- Kamera sökme / takma veya sıralama değişiklikleri sonrası:
  - Jetson yeniden başlatılmalı **veya**
  - ZED daemon yeniden başlatılmalıdır

    ```bash
    sudo systemctl restart zed_x_daemon
    ````

