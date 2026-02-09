# Linux Üzerinde ZED SDK Kurulumu

Bu doküman, **ZED SDK’nın Linux (Jetson dahil)** sistemlerde nasıl kurulacağını açıklar.

ZED SDK; kamera sürücüleri, kütüphaneler, araçlar ve örnek uygulamaları içerir.

---

## ZED SDK Nedir?

ZED SDK;

- Kamera kontrolü
- Derinlik algılama
- AI tabanlı nesne algılama
- Body Tracking
- CUDA hızlandırmalı görüntü işleme

özelliklerini sağlar.

---

## ZED SDK İndirme

Stereolabs resmi sitesinden **Linux için ZED SDK** indirin. https://www.stereolabs.com/en-tr/developers/release

İndirilen dosya genellikle şu formatta olur:

```text
ZED_SDK_UbuntuXX_cudaYY.Y_vZ.Z.Z.zstd.run
```

## Kurulum 

```bash
cd Downloads
sudo apt install zstd
```
```bash
chmod +x ZED_SDK_UbuntuXX_cudaYY.Y_vZ.Z.Z.zstd.run

./ZED_SDK_UbuntuXX_cudaYY.Y_vZ.Z.Z.zstd.run

```
