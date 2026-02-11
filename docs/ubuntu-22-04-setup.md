# NVIDIA Jetson İşletim Sistemi ve SDK Manager Yapılandırma Rehberi

Bu döküman, NeuRo-Lab bünyesinde kullanılan NVIDIA Jetson Orin NX/Nano modüllerinin ve A608 taşıyıcı kartlarının sistem kurulum süreçlerini kapsamaktadır. Rehber, hem resmi SDK Manager yöntemini hem de özel donanımlar için gerekli olan manuel flaşlama prosedürlerini içermektedir.

---

## 1. NVIDIA SDK Manager Kurulumu

NVIDIA SDK Manager, Jetson cihazlarına Linux for Tegra (L4T) işletim sistemini ve CUDA, cuDNN, TensorRT gibi temel kütüphaneleri yüklemek için kullanılan ana arayüzdür.

### 1.1. Ağ Deposu Üzerinden Kurulum (Önerilen)
Sistem depolarını güncel tutmak için terminal üzerinden aşağıdaki komutları icra ediniz. `[distro]` alanını işletim sistemi sürümünüze göre (örn: `ubuntu2204`) güncelleyiniz:

```bash
wget [https://developer.download.nvidia.com/compute/cuda/repos/](https://developer.download.nvidia.com/compute/cuda/repos/)[distro]/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install sdkmanager
```
### 1.2. Yerel Paket Üzerinden Kurulum

İndirilen .deb dosyası üzerinden kurulum yapmak için:

```bash
sudo apt install ./sdkmanager_[version]-[build#]_amd64.deb
```
## 2. JetPack 6.2 Manuel Kurulum (A608 Taşıyıcı Kart)

SDK Manager'ın stabil çalışmadığı durumlarda veya A608 gibi özel taşıyıcı kart konfigürasyonlarında sistemin manuel olarak flaşlanması gerekmektedir.
### 2.1. Donanım ve Yazılım Gereksinimleri

    Host PC: Fiziksel Ubuntu 20.04 veya 22.04 yüklü bilgisayar.

    Bağlantı: USB Type-C veri kablosu.

    Kurtarma Modu: Recovery header pinlerini kısa devre etmek için GH1.25MM kablo.

| Bileşen | Detay / Versiyon |
| :--- | :--- |
| **JetPack** | 6.2 (L4T 36.4.3) |
| **Modül** | Jetson Orin NX / Nano |
| **Taşıyıcı Kart** | A608 Carrier Board |


## 3. Dağıtım ve Flaşlama Prosedürü
###  3.1. Zorunlu Kurtarma Moduna (Force Recovery) Giriş

Cihazın flaşlanabilmesi için aşağıdaki adımlar takip edilerek Recovery Mode aktif edilmelidir:

1. Cihazın enerjisini tamamen kesin.

2. USB Type-C kablosu ile Host PC bağlantısını gerçekleştirin.

3. A608 kartı üzerindeki Recovery header'ında bulunan Pin 1 ve Pin 2'yi kısa devre yapın.

4. Cihaza güç verin.

5. Host PC terminalinde lsusb komutu ile cihazın tanındığını doğrulayın:
    ```bash
    lsusb | grep -i "NVIDIA Corp"
    ```
    
### 3.2. Dosya Sisteminin Hazırlanması

Sürücülerin ve kök dosya sisteminin (rootfs) Host PC üzerinde yapılandırılması:
Bash

#### Sürücü paketini ve rootfs'i açma
```bash
tar xf Jetson_Linux_R36.4.3_aarch64.tbz2
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
```
#### A608 çevre birimi sürücülerinin entegrasyonu
```bash
sudo tar zxpf 608_jp62.tar.gz
sudo cp -r 608_jp62/Linux_for_Tegra/* Linux_for_Tegra/
cd Linux_for_Tegra/
```
#### Bağımlılıkların ve ikili dosyaların uygulanması
```bash
sudo ./tools/l4t_flash_prerequisites.sh
sudo ./apply_binaries.sh
```
###  3.3. NVMe Üzerine Sistem Yükleme

Sistemi A608 üzerindeki NVMe depolama birimine yüklemek için aşağıdaki komutu çalıştırın:

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
-c tools/kernel_flash/flash_l4t_t234_nvme.xml \
-p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
--showlogs --network usb0 jetson-orin-nano-devkit-super internal
```
---

Not: Kurulum tamamlandıktan sonra cihaz yeniden başlayacaktır. İlk kurulum ekranında kullanıcı adı ve şifre yapılandırmasını tamamlayarak ROS 2 kurulum adımlarına geçebilirsiniz.

