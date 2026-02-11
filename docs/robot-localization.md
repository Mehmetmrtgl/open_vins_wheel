# ROS 2 Üzerinde robot_localization Kurulumu

Bu doküman, **ROS 2** kullanarak **robot_localization** paketinin kurulumu ve
temel kullanımına giriş yapmayı amaçlar.

`robot_localization`, IMU, tekerlek odometrisi, GPS gibi sensörlerden gelen
verileri birleştirerek (sensor fusion) robotun konumunu ve hızını hesaplayan
bir pakettir.

---

## 1. Desteklenen ROS 2 Dağıtımları

`robot_localization`, aşağıdaki ROS 2 sürümlerini destekler:

- ✅ ROS 2 Foxy
- ✅ ROS 2 Galactic
- ✅ ROS 2 Humble (önerilen)
- ✅ ROS 2 Iron

📌 **Not:** Aşağıdaki adımlar **ROS 2 Humble (Ubuntu 22.04)** temel alınarak yazılmıştır.

---

## 2. Ön Koşullar

Kuruluma başlamadan önce:

- ROS 2 kurulu olmalıdır
- ROS 2 ortamı source edilmiş olmalıdır

```bash
source /opt/ros/humble/setup.bash
```

## 3. robot_localization Paketinin Kurulumu

ROS 2 Humble için:
```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```

Kurulumun başarılı olduğunu kontrol etmek için:
```bash
ros2 pkg list | grep robot_localization
```
## 4. Workspace Üzerinden Kurulum (Opsiyonel)

Eğer paketi kaynak koddan derlemek istiyorsanız:

### Workspace Oluşturma
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Paketi Klonlama
```bash
git clone https://github.com/cra-ros-pkg/robot_localization.git
```

### Derleme
```bash
cd ~/ros2_ws
colcon build
```


### Derleme sonrası workspace’i source edin:
```bash

source ~/ros2_ws/install/setup.bash
```

## 5. Temel Çalıştırma Örneği
EKF Node’u Çalıştırma
```bash
ros2 run robot_localization ekf_node \
  --ros-args --params-file ekf.yaml
```


## Örnek Yaml Dosyası
```yaml
ekf_filter_node:
    ros__parameters:
        frequency: 50.0
        two_d_mode: true
        publish_tf: true
 
        map_frame: map             
        odom_frame: odom            
        base_link_frame: base_footprint
        world_frame: odom 

        #x     , y     , z,
        #roll  , pitch , yaw,
        #vx    , vy    , vz,
        #vroll , vpitch, vyaw,
        #ax    , ay    , az
        odom0: odom/unfiltered
        odom0_config: [false, false, false,
                       false, false, false,
                       true, true, false,
                       false, false, true,
                       false, false, false]

        imu0: imu/data
        imu0_config: [false, false, false,
                      false, false, false,
                      false, false, false,
                      false, false, true,
                      false, false, false]
```