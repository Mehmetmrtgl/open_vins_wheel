# ROS 1 Noetic Üzerinde Kalibr Kurulumu

Bu doküman, **Ubuntu 20.04 + ROS 1 Noetic** kullanarak  
**ETHZ-ASL Kalibr** aracının nasıl kurulacağını açıklar.

---

## Kalibr için Gerekli Sistem Bağımlılıkları

Aşağıdaki paketler Kalibr’in derlenmesi ve çalışması için gereklidir:

```bash
sudo apt install -y \
    git wget autoconf automake nano \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev
````

## Python Bağımlılıkları (Ubuntu 20.04)

```bash
sudo apt install -y \
    python3-dev python3-pip python3-scipy \
    python3-matplotlib ipython3 \
    python3-wxgtk4.0 python3-tk \
    python3-igraph python3-pyx
```

## Kalibr Projesini Klonlama
```bash
cd ~/kalibr_workspace/src
git clone https://github.com/ethz-asl/kalibr.git
```

##  Kalibr’i Derleme

Ana workspace dizinine dönün ve derlemeyi başlatın:
```bash
cd ~/kalibr_workspace
catkin build -DCMAKE_BUILD_TYPE=Release -j4
```
> Not:
Eğer sistem belleği sınırlıysa -j4 yerine -j2 kullanabilirsiniz.

##  Workspace’i Source Etme

Derleme tamamlandıktan sonra Kalibr’i kullanabilmek için:
```bash
source ~/kalibr_workspace/devel/setup.bash
````

## Kalibr Komutlarını Çalıştırma

