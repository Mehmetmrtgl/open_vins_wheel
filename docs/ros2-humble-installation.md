# ROS 2 Humble Hawksbill Kurulum 

Bu döküman, Ubuntu 22.04 (Jammy Jellyfish) işletim sistemi üzerinde çalışan NVIDIA Jetson cihazları için ROS 2 Humble Hawksbill sürümünün kurulum prosedürlerini içermektedir. Bu kurulum, laboratuvarımızdaki otonom navigasyon ve VIO çalışmaları için gerekli olan temel iletişim altyapısını sağlar.

---

## Ön Koşullar

Kuruluma başlamadan önce sistemin aşağıdaki gereksinimleri karşıladığından emin olunmalıdır:
* **İşletim Sistemi:** Ubuntu 22.04 LTS (JetPack 6.0+ ile uyumlu).
* **Donanım:** NVIDIA Jetson Orin serisi veya uyumlu ARM64 tabanlı sistemler.
* **Yetki:** Sudo ayrıcalıklarına sahip kullanıcı hesabı.

---

## 1. Yerel Ayarların (Locale) Yapılandırılması

ROS 2, UTF-8 destekli bir yerel ayar gerektirir. Sistem dil ayarlarını aşağıdaki komutlarla standardize ediniz:

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. ROS 2 Depolarının Sisteme Tanımlanması

Resmi ROS 2 paketlerine erişebilmek için GPG anahtarını ve depo adresini kaynak listenize ekleyiniz:

```bash
# Gerekli araçların kurulumu
sudo apt update && sudo apt install -y curl gnupg lsb-release

# ROS 2 GPG anahtarının sisteme eklenmesi
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

# Depo adresinin kaynak listesine yazılması
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
# 3. Paket Kurulumu

Sistem paket indeksini güncelleyerek, görselleştirme araçlarını da içeren masaüstü sürümünü kurunuz:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```


# 4. Çevresel Değişkenlerin Yapılandırılması

ROS 2 komut setinin her terminal oturumunda otomatik olarak yüklenebilmesi için setup.bash dosyasını kullanıcı profilinize ekleyiniz:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# 5. Kurulum Doğrulaması

Kurulumun başarılı olduğunu teyit etmek amacıyla, iki ayrı terminal üzerinden haberleşme testi gerçekleştiriniz:

Terminal 1 (Yayıncı):

```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2 (Dinleyici):

```bash
ros2 run demo_nodes_cpp listener
```

# 6. Geliştirme Araçları ve Bağımlılık Yönetimi

NeuRo-Lab bünyesindeki özel paketlerin derlenmesi ve bağımlılıkların otomatik çözülmesi için colcon ve rosdep araçlarını yapılandırınız:

```bash
# Derleme ve yönetim araçlarının kurulumu
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Rosdep veritabanının ilklendirilmesi ve güncellenmesi
sudo rosdep init
rosdep update
```

