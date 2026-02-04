# Sistem Kurulumu ve Hazırlık Rehberi

NeuRo-Lab bünyesinde yürütülen çalışmalarda, NVIDIA Jetson platformunun (Orin NX/Nano) temel yazılım katmanlarını yapılandırmak için aşağıdaki adımlar sırasıyla takip edilmelidir. Bu bölüm; donanım yazılımının yüklenmesi, grafik işlem birimi (GPU) kütüphanelerinin yapılandırılması ve ROS 2 ekosisteminin kurulması süreçlerini içermektedir.



---

## Kurulum ve Yapılandırma Modülleri

Aşağıdaki dökümanlar, sistemin sıfırdan OpenVINS çalıştırabilir hale getirilmesi için gerekli olan tüm teknik prosedürleri adım adım sunmaktadır:

### 1. İşletim Sistemi ve SDK Manager Yapılandırması
NVIDIA Jetson cihazlarının flaşlanması, A608 taşıyıcı kartı sürücülerinin entegrasyonu ve Host PC üzerinden sistem yükleme süreçlerini içerir.
* [Jetson İşletim Sistemi ve Flashing Rehberi](ubuntu-22-04-setup.md)

### 2. CUDA ve Grafik Kütüphaneleri
VIO algoritmalarının GPU üzerinde paralel hesaplama yapabilmesi için gerekli olan CUDA Toolkit, cuDNN ve JetPack bileşenlerinin kontrolü ve kurulum süreçlerini içerir.
* [CUDA Yapılandırma Rehberi](cuda-setup.md)

### 3. ROS 2 Humble Ekosistemi
Ubuntu 22.04 tabanlı sistemlerde ROS 2 Humble Hawksbill sürümünün kurulumu, ortam değişkenlerinin ayarlanması ve geliştirme araçlarının yapılandırılmasını içerir.
* [ROS 2 Humble Kurulum Rehberi](ros2-humble-installation.md)

---

**Not:** Kurulum sürecinde tutarlılığı sağlamak adına, önce işletim sistemi yüklemesinin tamamlanması, ardından CUDA ve ROS 2 kurulumlarına geçilmesi teknik olarak tavsiye edilmektedir.
