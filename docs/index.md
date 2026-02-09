# NeuRo-Lab

Hacettepe Üniversitesi Elektrik ve Elektronik Mühendisliği Bölümü bünyesinde, Doç. Dr. İsmail Uyanık liderliğinde yürütülen bu çalışma alanı; GPS bağımsız otonom navigasyon, sensör füzyonu ve Visual-Inertial Odometry (VIO) algoritmalarının geliştirilmesine odaklanmaktadır.

Bu dokümantasyon, laboratuvarımızda yürütülen VINS çalışmalarının kurulum, kalibrasyon ve uygulama aşamalarını içeren teknik bir rehber niteliğindedir.

---

## 1. Sistem Gereksinimleri ve Hazırlık
Navigasyon algoritmalarının kararlı çalışabilmesi için gerekli temel yazılım katmanları ve ortam kurulumları.

* [İşletim Sistemi ve ROS 2 Humble Kurulumu](env-setup.md)
* [OpenVINS Kurulumu ve Temel Kütüphaneler (Eigen, OpenCV, Ceres)](openvins-ros2-install.md)

## 2. Sensör Sisteminin Ros2 Sisteminde çalıştırılması
* [ZEDX Kamera ve ZEDX Capture Card](zed-link-mono-orin-nano.md)
* [Xsens IMU](Xsens-IMU.md)


## 3. Sensör Kalibrasyon Süreçleri
VIO algoritmalarında yüksek hassasiyet için gereken kamera ve IMU kalibrasyon metodolojileri.

* [IMU Gürültü Karakterizasyonu ve Allan Variance Analizi](allan-variance.md)
* [Kalibr Toolbox ile Kamera-IMU Kalibrasyonu](kalibr.md)

## 4. Tekerlekli Robot Sistemin Çalıştırılması

Bu bölümde, 4 tekerlekli mobil robot sisteminin çalıştırılması için gerekli olan
temel yazılım bileşenlerinin kurulumu ve yapılandırılması ele alınmaktadır.

Aşağıdaki paketlerin kurulmuş ve doğru şekilde yapılandırılmış olması gerekmektedir:

* [micro-ROS Kurulumu](mirco-ros.md)

* [Robot Localization Paketi Kurulumu](robot-localization.md)
## 5. Rapor ve Analizler
Tahmin sonuçlarının doğrulanması ve performans ölçümü.

* [EVO Aracı ile Yörünge Analizi (RPE/ATE)](evo.md)
---

**İletişim ve Katkı:**
Bu dokümantasyon NeuRo-Lab araştırmacıları için bir rehber niteliğindedir. Teknik sorularınız, hata bildirimleri ve katkılarınız için depo yöneticisi ile iletişime geçiniz.
