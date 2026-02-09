# OpenVINS Kurulum Rehberi

Bu doküman, OpenVINS algoritmasının ROS 2 (Humble/Foxy) ortamında sıfırdan kurulumunu kapsar.

##  Gereksinimler
* **Ubuntu:** 22.04 (Humble için)
* **ROS 2:** Humble Hawksbill (Önerilen)
* **OpenCV:** 3.2+
* **Eigen:** 3.3+

##  Kurulum Adımları

### 1. Gerekli Kütüphanelerin Yüklenmesi
`libeigen3-dev`, doğrusal cebir işlemleri (matrisler, vektörler) için hafif ve hızlı bir şablon kütüphanesidir. `libboost-all-dev`, Boost’un tüm geliştirme paketlerini içerir ve dosya sistemi, çok iş parçacığı, tarih/saat gibi pek çok yardımcı bileşen sağlar. `libceres-dev` ise optimizasyon problemleri (özellikle doğrusal olmayan en küçük kareler) için kullanılan Google Ceres Solver kütüphanesini sunar. 

```bash
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
```

> **Note:** Eğer ROS 2 yüklü değilse, önce [ROS 2 Kurulum Dokümanı](ros2-humble-installation.md) dosyasını inceleyin.

### 2. Çalışma Alanının Oluşturulması
Öncelikle ROS 2 workspace'inizi oluşturun:
```bash
mkdir -p ~/workspace/catkin_ws_ov/src/
cd ~/workspace/catkin_ws_ov/src/
```
### 3. Kaynak Kodun İndirilmesi

OpenVINS reposunu ve gerekli bağımlılıkları klonlayın:

```bash
git clone [https://github.com/rpng/open_vins.git](https://github.com/rpng/open_vins.git)
```

### 4. Bağımlılıkların Yüklenmesi

ROS 2 bağımlılıklarını rosdep ile çözün:
```bash
cd ~/workspace/catkin_ws_ov
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Derleme

Projeyi derlemek için colcon kullanın:
```bash
colcon build --symlink-install --paralel-workers 1
```

###  6. Çalıştırma

Kurulum bittikten sonra workspace'i kaynak olarak ekleyin ve örnek bir veri seti ile başlatın:

```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

###  7. Ek Değerlendirme Gereklilikleri

```bash
sudo apt-get install python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk # for python3 systems
```

### 8. OpenCV (kaynaktan)

> **Note:** 
Öncelikle sisteminiz / ROS OpenCV ile derlemeyi deneyin. OpenVINS ekipi tarafından OpenCV 3.2, 3.3, 3.4, 4.2 ve 4.5 ile derleme test edildiği bildirilmiştir. Derleme yapamıyorsanız veya daha yeni bir sürüm istiyorsanız, ancak bu yöntemi kullanın!

```bash
git clone https://github.com/opencv/opencv/
git clone https://github.com/opencv/opencv_contrib/
mkdir opencv/build/
cd opencv/build/
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j8
sudo make install
```