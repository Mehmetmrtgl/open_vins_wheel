# Görsel-Eylemsiz Kilometre Ölçümü (VIO) Genel Bakış

Visual-Inertial Odometry (VIO), bir robotun veya insansız aracın konumunu ve yönelimini (pose), sadece bir kamera (visual) ve bir Atalet Ölçüm Birimi (IMU - inertial) kullanarak tahmin etme sürecidir. GPS'in olmadığı veya sinyalinin zayıf olduğu ortamlarda (kapalı alanlar, ormanlık bölgeler, kanyonlar) kritik öneme sahiptir.

---

## Neden Kamera ve IMU?

Kamera ve IMU, birbirlerinin zayıf yönlerini tamamlayan sensörlerdir:
Özellik	Kamera (Görsel)	IMU (Eylemsiz)
Örnekleme Hızı	Düşük (15-60 Hz)	Yüksek (200-1000 Hz)
Hareket Tahmini	Yavaş hareketlerde başarılı	Hızlı/Ani hareketlerde başarılı
Hata Birikimi	Uzun vadede drift (kayma) yapar	Çok hızlı drift yapar
Ölçek (Scale)	Tek kamera ile ölçek belirsizdir	Ölçek bilgisini sağlar (Yerçekimi sayesinde)

---

##  Tahmin Yöntemleri

VIO algoritmaları temel olarak iki ana yaklaşıma ayrılır. NeuRo-Lab bünyesinde biz daha çok Filtreleme tabanlı yaklaşımlara odaklanmaktayız.
### 1. Filtreleme Tabanlı (EKF / MSCKF)

Bu yöntem, sistemin durumunu bir Kalman Filtresi kullanarak günceller.

    MSCKF (Multi-State Constraint Kalman Filter): OpenVINS'in de temelini oluşturan bu algoritma, görsel öznitelikleri (features) durum vektörüne eklemeden, kamera pozları üzerinden kısıtlar oluşturarak çalışır. Bu sayede hesaplama maliyeti düşük kalır.

### 2. Optimizasyon Tabanlı (Graph-based / Bundle Adjustment)

Geçmişteki tüm pozları ve öznitelikleri bir çizge (graph) üzerinde optimize eder.

    Örnekler: VINS-Mono, OKVIS, ORB-SLAM3.

    Avantaj: Daha yüksek doğruluk.

    Dezavantaj: Yüksek hesaplama gücü gereksinimi.
---
## Matematiksel Temel

Bir **Visual-Inertial Odometry (VIO)** sisteminde, aracın anlık durumunu temsil eden durum vektörü ($x$), genellikle dünya çerçevesi (world frame) ve gövde çerçevesi (body frame) arasındaki ilişkiyi tanımlayan şu bileşenlerden oluşur:

$$x = \begin{bmatrix} q_b^w & p_b^w & v_b^w & b_a & b_g \end{bmatrix}^T$$



### Bileşenlerin Açıklaması

* **$q_b^w$ (Yönelim):** Gövde çerçevesinden dünya çerçevesine olan dönüşümü temsil eden birim kuaterniyon (Quaternion). Aracın uzaydaki 3 eksenli rotasyonunu tanımlar.
* **$p_b^w$ (Konum):** Aracın dünya çerçevesindeki 3 boyutlu konum vektörüdür ($x, y, z$).
* **$v_b^w$ (Hız):** Aracın dünya çerçevesindeki lineer hız vektörüdür.
* **$b_a$ (İvmeölçer Sapması):** IMU ivmeölçer sensöründeki düşük frekanslı gürültü veya kayma (bias) değeridir.
* **$b_g$ (Jiroskop Sapması):** IMU jiroskop sensöründeki açısal hız sapması (bias) değeridir.

---

**Not:** Bu vektör, MSCKF gibi filtreleme tabanlı yaklaşımlarda her zaman adımı (timestamp) için güncellenir. IMU verileriyle tahmin (prediction) yapılırken, kamera verileriyle düzeltme (update) işlemi gerçekleştirilir.

## NeuRo-Lab Çalışma Alanı

Laboratuvarımızda, OpenVINS platformu üzerinde şu geliştirmelere odaklanıyoruz:

    Tekerlek Odometrisi Entegrasyonu: Robot sistemimizden gelen odometri mesajını kullanarak preintegration (ön-tümleme) yöntemleri.

    Zorlu Koşullarda Dayanıklılık: Düşük ışık veya hızlı hareket durumlarında VIO kararlılığı.

    Donanım Optimizasyonu: Algoritmaların gömülü sistemlerde (Jetson, NUC) gerçek zamanlı çalıştırılması.

---
