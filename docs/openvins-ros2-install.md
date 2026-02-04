# OpenVINS ROS 2 Kurulum Rehberi

Bu doküman, OpenVINS algoritmasının ROS 2 (Humble/Foxy) ortamında sıfırdan kurulumunu kapsar.

## 📋 Gereksinimler
* **Ubuntu:** 22.04 (Humble için)
* **ROS 2:** Humble Hawksbill (Önerilen)
* **OpenCV:** 3.2+
* **Eigen:** 3.3+

## 🛠️ Kurulum Adımları

### 1. Çalışma Alanının Oluşturulması
Öncelikle ROS 2 workspace'inizi oluşturun:
```bash
mkdir -p ~/ov_ws/src
cd ~/ov_ws/src
```
### 2. Kaynak Kodun İndirilmesi

OpenVINS reposunu ve gerekli bağımlılıkları klonlayın:

```bash
git clone [https://github.com/rpng/open_vins.git](https://github.com/rpng/open_vins.git)
```

### 3. Bağımlılıkların Yüklenmesi

ROS 2 bağımlılıklarını rosdep ile çözün:
```bash
cd ~/ov_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Derleme

Projeyi derlemek için colcon kullanın:
```bash
colcon build --symlink-install --paralel-workers 1
```

###  Çalıştırma

Kurulum bittikten sonra workspace'i kaynak olarak ekleyin ve örnek bir veri seti ile başlatın:

```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```
