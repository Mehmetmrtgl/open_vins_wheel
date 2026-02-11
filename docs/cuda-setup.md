# CUDA Yapılandırma ve Grafik Kütüphaneleri Doğrulama Rehberi

NVIDIA Jetson platformunda yürütülen VIO (Visual-Inertial Odometry) çalışmaları, yüksek yoğunluklu paralel hesaplama gücüne ihtiyaç duyar. Bu döküman, sistemdeki CUDA ekosisteminin yönetimi ve doğrulanması süreçlerini kapsar.

---

## 1. Genel Yapılandırma Hakkında Not

NVIDIA Jetson cihazlarında **CUDA Toolkit**, **JetPack SDK**'nın ayrılmaz bir parçası olarak işletim sistemiyle birlikte önceden yapılandırılmış (pre-installed) şekilde sunulur. Bu nedenle, standart masaüstü bilgisayarların aksine manuel bir sürücü veya toolkit kurulumuna ihtiyaç duyulmaz.

---

## 2. JetPack Bileşenlerinin Kurulumu ve Onarımı

Sistemdeki CUDA kütüphanelerinin eksik olması veya bozulması durumunda, tüm JetPack bileşenlerini (CUDA, cuDNN, TensorRT) aşağıdaki komutlarla onarabilir veya en güncel sürüme yükseltebilirsiniz:

```bash
sudo apt update
sudo apt install nvidia-jetpack
```

This will install or reinstall the full JetPack SDK, including:

- CUDA Toolkit
- cuDNN
- TensorRT
- and other necessary components

## 3. Kurulumun Doğrulanması

CUDA derleyicisinin sistem yollarında (PATH) tanımlı olduğunu ve GPU ile iletişim kurabildiğini doğrulamak için aşağıdaki komutu kullanın:

```bash
nvcc --version
```

Eğer bu komut çalışmıyorsa, çevre değişkenlerini .bashrc dosyanıza eklemeniz gerekebilir:
```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```


