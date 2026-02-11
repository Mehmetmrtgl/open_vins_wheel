# IMU Gürültü Karakterizasyonu ve Allan Variance Analizi

Bu doküman, ROS ile çalışan **Allan Variance ROS** aracı kullanılarak  
**Inertial Measurement Unit (IMU)** sensörlerindeki gürültü ve hata parametrelerinin  
nasıl analiz edileceğini açıklar. Analiz, **Allan Variance / Allan Deviation** metoduna dayanır.

Araç, IMU verilerini okuyan ve bunlardan aşağıdaki önemli gürültü parametrelerini çıkaran bir ROS paketidir: :contentReference[oaicite:0]{index=0}

---

## Kaynak Kod

Bu analiz aracı aşağıdaki GitHub deposunda bulunabilir:

> https://github.com/ori-drs/allan_variance_ros 

## Paket Özellikleri

- ROS1 ile tamamen uyumlu  
- `.rosbag` formatındaki IMU verilerini doğrudan işler  
- C++ ile performanslı bir işleyiş sunar  
- Kalibrasyon için **imu.yaml** dosyası üretir  
- Kalibrasyon araçlarıyla (örneğin Kalibr) birlikte çalışabilir 

## 🛠️ Yapılandırma ve Derleme

Allan Variance ROS aracı aşağıdaki şekilde derlenir:

```bash
# Catkin workspace içinde
cd ~/catkin_ws
catkin build allan_variance_ros
source devel/setup.bash
```
roscore'u çalıştır
```bash
roscore
```

## Çalıştırma

Aşağıdaki adımlar, **allan_variance_ros** paketi kullanılarak IMU verilerinin
Allan Variance / Allan Deviation analizinin yapılmasını açıklar.

---

### 1. IMU Verisinin Kaydedilmesi
IMU sensörünü titreşimi sönümlenmiş sabit bir yüzeye yerleştirin. IMU’dan gelen verileri bir rosbag dosyasına kaydedin.

> **Note:** Eğer ros2 sisteminde çalışıyorsanız ros2 bag formatında kaydettikten sonra rosbag formatına dönüştürün.

Minimum kayıt süresi: 3 saat

> **Öneri:** Kayıt süresi ne kadar uzun olursa, sonuçlar o kadar doğru olur

### 2. ROS Mesajlarını Zaman Damgasına Göre Düzenleme (Önerilen)
IMU mesajlarını zaman sırasına göre yeniden düzenlemek için aşağıdaki komutu kullanın:

```bash
rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag
```

original_rosbag → Ham IMU verisini içeren rosbag

cooked_rosbag → Zaman sıralı, analiz için hazır rosbag

### 3. Allan Variance Hesaplamasını Çalıştırma
Hazırlanan rosbag dosyası ile Allan Variance hesaplamasını başlatın:

```bash
rosrun allan_variance_ros allan_variance [rosbag_klasör_yolu] [config_dosyası_yolu]
```

IMU için Allan Deviation hesaplar.

Sonuçları CSV formatında bir dosya olarak üretir.

### 4. Sonuçların Analizi ve Görselleştirme
Oluşturulan CSV dosyasını analiz etmek ve grafiklerini görmek için:

```bash
rosrun allan_variance_ros analysis.py --data allan_variance.csv
```

Allan Deviation grafikleri çizilir

Gürültü parametreleri otomatik olarak çıkarılır.

### 5 Opsiyonel: Config Dosyası ile Analiz
Eğer IMU topic adı ve güncelleme frekansını belirten bir config dosyanız varsa,
analiz komutuna ekleyebilirsiniz:

```bash
rosrun allan_variance_ros analysis.py \
  --data allan_variance.csv \
  --config config/xsens.yaml
```

Farklı IMU’lar için daha doğru parametre çıkarımı sağlar.

Sensöre özel ayarların kullanılmasına olanak tanır.

> **NOTE** xsens.yaml default olarak gelmemekte kendiniz oluşturmalısınız.

