# evo – Trajectory Değerlendirme Aracı

**evo**, SLAM, VO ve VIO sistemlerinden elde edilen **trajektoryaları**
karşılaştırmak ve değerlendirmek için kullanılan açık kaynaklı bir araçtır.

---

## Ne İşe Yarar?

evo ile:

- Gerçek referans (ground truth) ile tahmin edilen trajektoryalar karşılaştırılır
- Konum hataları sayısal olarak ölçülür
- Trajektoryalar görselleştirilir

## Kurulum

```bash
pip install evo
```

## Metrikler:

```evo_ape``` - mutlak poz hatası

```evo_rpe``` - göreceli poz hatası

## Araçlar:

```evo_traj``` - bir veya daha fazla yörüngeyi analiz etmek, çizmek veya dışa aktarmak için kullanılan araç

```evo_res``` - evo_ape veya evo_rpe'den bir veya daha fazla sonuç dosyasını karşılaştırmak için kullanılan araç

```evo_config``` - genel ayarlar ve yapılandırma dosyası işlemleri için kullanılan araç

# Kullanım için
- [Formatlar için ](
https://github.com/MichaelGrupp/evo/wiki/Formats)

- [evo_traj için ](https://github.com/MichaelGrupp/evo/wiki/evo_traj)

- [Metrikler için](https://github.com/MichaelGrupp/evo/wiki/Metrics)