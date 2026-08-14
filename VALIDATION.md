# Doğrulama özeti

Kaynak paket oluşturulurken aşağıdaki denetimler uygulanmıştır:

- 20 MATLAB kaynak dosyasının blok (`classdef`, `methods`, `function`, `if`, `for`, `end`) dengesi ve birincil dosya–deklarasyon adları statik olarak kontrol edildi.
- Projede yalnızca bir açık `for` döngüsü bulunduğu doğrulandı: `simulation/runEVSimulation.m` içindeki ana RK4 zaman döngüsü.
- Yarış çevriminin zaman ve hız düğüm dizilerinin ikisinin de 79 elemanlı olduğu doğrulandı.
- Üretilen profilin 60.001 örnek, 0–600 s kapsama ve 100 Hz örnekleme içerdiği; hız aralığının 0–170 km/h olduğu sayısal olarak kontrol edildi.
- Profil türevinin yaklaşık −6.76 ile +4.22 m/s² arasında olduğu kontrol edildi.
- PMSM sınıfındaki Clarke/Park ileri–geri dönüşümünün bağımsız sayısal kontrolünde en büyük hata yaklaşık `4.3e-14 A` bulundu.
- Başlangıç seri-grup OCV aralığı yaklaşık 4.1488–4.1634 V, başlangıç paket OCV’si yaklaşık 399.0 V bulundu.
- Varsayılan 96s konfigürasyonun birleşik sürekli durum boyutu 393 olarak doğrulandı.
- Sondaki 600.000 s sürüş çevrimi sorgusunun son örneği doğru döndürmesi için uç nokta testi eklendi.

Bu çalışma ortamında MATLAB veya GNU Octave yürütücüsü bulunmadığından `run_smoke_tests` burada çalıştırılamadı. Teslim paketi, MATLAB üzerinde gerçek yürütme için `tests/run_smoke_tests.m` dosyasını içerir. Bu test 2.000 adet 10 µs RK4 adımı, LUT pozitifliği, durum boyutu, sonlu değerler, paket gerilimi ve dönüşüm matrislerini denetler.
