# NeuRo-Lab

Hacettepe Üniversitesi Elektrik ve Elektronik Mühendisliği Bölümü bünyesinde, Doç. Dr. İsmail Uyanık liderliğinde yürütülen bu çalışma alanı; GPS bağımsız otonom navigasyon, sensör füzyonu ve Visual-Inertial Odometry (VIO) algoritmalarının geliştirilmesine odaklanmaktadır.

Bu dokümantasyon, laboratuvarımızda yürütülen VINS çalışmalarının kurulum, kalibrasyon ve uygulama aşamalarını içeren teknik bir rehber niteliğindedir.

---

## 1. Sistem Gereksinimleri ve Hazırlık
Navigasyon algoritmalarının kararlı çalışabilmesi için gerekli temel yazılım katmanları ve ortam kurulumları.

* [İşletim Sistemi ve ROS 2 Humble Kurulumu](env-setup.md)
* [Temel Kütüphaneler ve Bağımlılıklar (Eigen, OpenCV, Ceres)]


## 2. Sensör Kalibrasyon Süreçleri
VIO algoritmalarında yüksek hassasiyet için gereken kamera ve IMU kalibrasyon metodolojileri.

* [Kalibr Toolbox ile Kamera-IMU Kalibrasyonu]
* [İçsel (Intrinsic) ve Dışsal (Extrinsic) Parametrelerin Belirlenmesi]
* [IMU Gürültü Karakterizasyonu ve Allan Variance Analizi]

## 3. VIO Algoritmaları ve Uygulama
Laboratuvarımızda kullanılan ana algoritmalar ve yapılandırma parametreleri.

* [OpenVINS ROS 2 Kurulum ve Konfigürasyon Rehberi](openvins-ros2-install.md)
* [Multi-State Constraint Kalman Filter (MSCKF) Teorik Temelleri](vio-bilgi.md)
* [Veri Setleri (EuRoC, Kaist Urban) ile Test ve Benchmarking]

## 4. Kinematik Modelleme ve Veri Seti İşleme
Özel veri setleri için gereken kinematik modeller ve mesaj dönüşümleri.

* [Ackermann Direksiyon Sistemi Kinematik Modeli](
* [Kaist Urban Veri Seti için Odometri Mesajı Üretimi]

## 5. Odometri ve Sensör Entegrasyonu
Tekerlek odometrisi verilerinin ana algoritmaya füzyonu ve preintegration teknikleri.

* [Tekerlek Odometrisi ve Preintegration Entegrasyonu]

## 6. Analiz, Görselleştirme ve Değerlendirme
Tahmin sonuçlarının doğrulanması ve performans ölçümü.

* [EVO Aracı ile Yörünge Analizi (RPE/ATE)]
* [RViz 2 Görselleştirme ve Hata Ayıklama Ayarları]

---

**İletişim ve Katkı:**
Bu dokümantasyon NeuRo-Lab araştırmacıları için bir rehber niteliğindedir. Teknik sorularınız, hata bildirimleri ve katkılarınız için depo yöneticisi ile iletişime geçiniz.
