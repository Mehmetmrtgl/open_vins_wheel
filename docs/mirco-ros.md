
# Workspace oluştur ve içine gir
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
```

## micro-ROS build sistemini indir
```bash
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
````

## Bağımlılıkları kontrol et ve yükle
```bash
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y
````

# Build Sistemini Derleme
İndirdiğin paketi derleyerek micro-ROS'un yönetim araçlarını hazır hale getirmelisin:

```bash
colcon build
source install/local_setup.bash
```

# micro-ROS Agent Kurulumu
Bilgisayarının mikrodenetleyici ile konuşabilmesi için Agent'ı kurmalısın:

```bash
# Agent paketlerini indir ve oluştur
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

#  micro-ROS Agent'ı Çalıştırma
Her şey hazır olduğunda, mikrodenetleyicini USB ile bağlayıp (veya WiFi üzerinden) Agent'ı başlatarak haberleşmeyi sağlayabilirsin:

```bash
# Seri port üzerinden bağlantı için (Örn: /dev/ttyUSB0)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_dev -b 921600
```
