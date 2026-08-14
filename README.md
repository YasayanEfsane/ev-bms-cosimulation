# İleri Seviye Otonom EV Güç Aktarım ve BMS Co-Simülasyonu

Bu proje; yüksek performanslı bir Formula Student / TEKNOFEST elektrikli yarış aracının boylamsal dinamiğini, PMSM–FOC sürücüsünü, 96s10p batarya paketini ve sıvı soğutma sistemini tek bir birleşik durum vektörü üzerinde çözen saf MATLAB uygulamasıdır. Simulink, Simscape ve hazır blok diyagram kullanılmaz. Sürekli dinamiklerin tamamı sabit adımlı klasik RK4 ile koşturulur.

Paket, 96 seri ve her seride 10 paralel hücreden oluşur. BMS açısından her paralel grup ayrı bir ölçüm kanalıdır; bu nedenle model 960 fiziksel hücreyi 96 ayrı seri-grup eşdeğeriyle temsil eder. Hücre üst gerilimi 4.20 V olduğundan paket üst gerilimi 403.2 V’tur ve bu, otomotivdeki “400 V sınıfı” tanımıyla uyumludur. Hücre kapasitesi 80 kWh hedefinden otomatik türetilir (yaklaşık 22.52 Ah/hücre).

## Hızlı başlangıç

MATLAB R2021b veya sonrası önerilir. Ek toolbox gerekmez.

```matlab
cd('EV_BMS_CoSimulation')
simulationPreset = "smoke";
run('main_simulation.m')
```

Mevcut çalışma kipleri:

| Kip | Süre | RK4 adımı | Amaç |
|---|---:|---:|---|
| `smoke` | 0.20 s | 20,000 | Kurulum ve mimari kontrolü |
| `short` | 10 s | 1,000,000 | Kısa mühendislik önizlemesi |
| `full` | 600 s | 60,000,000 | İstenen tam yarış çevrimi |

`main_simulation.m`, değişken tanımlanmadığında `full` kipini kullanır. Tam koşu dört türev değerlendirmesi/adım nedeniyle 240 milyon birleşik model değerlendirmesi yapar; işlemciye bağlı olarak uzun sürebilir. Entegrasyon her kipte **10 µs** kalır. Belleği denetlemek için yalnızca 100 Hz çıktılar kaydedilir; hızlı FOC durumları yine 10 µs adımla çözülür.

Doğrulama için:

```matlab
setupProjectPath;
report = run_smoke_tests();
```

## Dosya mimarisi

```text
EV_BMS_CoSimulation/
├── main_simulation.m
├── setupProjectPath.m
├── components/
│   ├── @DynamicComponent/DynamicComponent.m
│   ├── @VehicleDynamics/VehicleDynamics.m
│   ├── @PMSMMotor/PMSMMotor.m
│   ├── @BatteryPack/BatteryPack.m
│   └── @ThermalManager/ThermalManager.m
├── system/@EVPowertrainSystem/EVPowertrainSystem.m
├── simulation/
│   ├── runEVSimulation.m
│   ├── initializeSimulationLog.m
│   ├── recordSimulationLog.m
│   └── trimSimulationLog.m
├── solvers/rk4Step.m
├── functions/
│   ├── defaultEVConfig.m
│   ├── validateEVConfig.m
│   ├── buildEVSystem.m
│   ├── generateRaceDriveCycle.m
│   ├── sampleDriveCycle.m
│   └── plotSimulationResults.m
└── tests/run_smoke_tests.m
```

`DynamicComponent`, tüm fiziksel sınıfların temel sınıfıdır. `buildEVSystem.m` composition root olarak sınıfları oluşturur ve constructor dependency injection ile `EVPowertrainSystem` nesnesine verir. Fiziksel sınıflar birbirlerini kendileri oluşturmaz. Projedeki tek açık `for` döngüsü `runEVSimulation.m` içindeki RK4 zaman döngüsüdür; hücre/grup işlemleri vektörizedir.

## Birleşik durum vektörü

Varsayılan 96s konfigürasyonda (X\in\mathbb{R}^{393}):

| Aralık (MATLAB, 1 tabanlı) | Durum | Boyut |
|---:|---|---:|
| 1 | Araç boylamsal hızı (v_x) | 1 |
| 2–7 | (i_d,i_q), hız PI integrali, d/q akım PI integralleri, θₑ | 6 |
| 8–103 | RC-1 kutup gerilimleri | 96 |
| 104–199 | RC-2 kutup gerilimleri | 96 |
| 200–295 | Seri-grup SoC değerleri | 96 |
| 296–391 | Seri-grup sıcaklıkları | 96 |
| 392 | Motor sıcaklığı | 1 |
| 393 | Soğutucu sıcaklığı | 1 |

BMS’nin bleeding maskesi ve aktif transfer komutu sürekli fiziksel durum değildir; 100 ms’de bir hesaplanan ve bir sonraki BMS tick’ine kadar tutulan ayrık kontrol çıktılarıdır. RK4’ün dört alt değerlendirmesinde değiştirilmezler.

## Model denklemleri

### Araç

Teker kuvveti ve dirençler:

\[
F_t=\frac{T_m i_g \eta_g}{r_w},\qquad
F_d=\tfrac12\rho C_d A v|v|,
\]

\[
F_{rr}=m g C_{rr}\tanh(v/v_\epsilon)\cos\alpha,\qquad
F_g=mg\sin\alpha.
\]

Dönen tekerler ve motor rotoru eşdeğer kütleye eklenir:

\[
m_{eq}=m+\frac{N_wJ_w}{r_w^2}+\frac{J_m i_g^2}{r_w^2},\qquad
\dot v=\frac{F_t-F_d-F_{rr}-F_g}{m_{eq}}.
\]

### PMSM, FOC ve field weakening

\[
\dot i_d=\frac{v_d-R_si_d+\omega_eL_qi_q}{L_d},\qquad
\dot i_q=\frac{v_q-R_si_q-\omega_e(L_di_d+\psi_f)}{L_q}.
\]

\[
T_e=\tfrac32p\left[\psi_f i_q+(L_d-L_q)i_di_q\right].
\]

Hız PI’si tork referansını üretir. Baz hız üstünde DC bara gerilim sınırından ve hız oranından negatif (i_d^*) hesaplanır; kalan akım çemberi (i_q^*) için kullanılır. Clarke ve Park dönüşümleri `abcToDq` / `dqToAbc` içinde açık matris çarpımlarıdır. d/q akım PI’larında çapraz bağ giderimi ve gerilim vektörü doyumu vardır.

Bakır, demir, windage, inverter iletim ve anahtarlama kayıpları DC güç hesabına eklenir. Rejenerasyonda aynı enerji işareti korunur: (P_{dc}=P_{mech}+P_{loss}).

### 2-RC Thevenin batarya

Her seri grup için:

\[
\dot V_1=\frac{I_g}{C_1}-\frac{V_1}{R_1C_1},\qquad
\dot V_2=\frac{I_g}{C_2}-\frac{V_2}{R_2C_2},
\]

\[
V_t=V_{oc}(SoC,T)-V_1-V_2-I_gR_0,
\qquad
\dot{SoC}=-\frac{I_{eff}}{3600Q_g}.
\]

(V_{oc},R_0,R_1,C_1,R_2,C_2) iki boyutlu SoC–sıcaklık LUT’larından önceden kurulan doğrusal `griddedInterpolant` nesneleriyle her türev değerlendirmesinde bulunur. LUT yüzeyleri doğrusal olmayan fiziksel eğilimler içerir; sorgu aralık dışında güvenli biçimde sınırlandırılır.

Motor–batarya cebirsel bağı kapalı biçimde çözülür:

\[
P=(E-IR)I
\quad\Rightarrow\quad
I=\frac{2P}{E+\sqrt{E^2-4RP}}.
\]

Sonuç hücre gerilimi, şarj/deşarj akımı ve maksimum güç sınırlarına tabi tutulur.

### BMS dengeleme

BMS 100 ms’de bir seri-grup terminal gerilimlerini tarar. `startVoltageSpreadV` / `stopVoltageSpreadV` histerezisi chatter oluşmasını önler. Varsayılan `passive` kipte yüksek gruplar 18 Ω bleeding direncine bağlanır. Direnç ısısı doğrudan termal modele gider. Yapı ayrıca aşağıdaki kipleri destekler:

```matlab
cfg.bms.mode = 'passive'; % varsayılan
cfg.bms.mode = 'active';
cfg.bms.mode = 'hybrid';
```

Aktif kipte yüksek gruplardan düşük gruplara vektörize enerji transferi yapılır ve `activeEfficiency` kaybı ısıya eklenir.

### Termal yönetim

\[
C_{th,c}\dot T_c=\dot Q_c-\frac{T_c-T_f}{R_{th,c}(\dot m)},
\]

\[
C_f\dot T_f=\sum\frac{T_c-T_f}{R_{th,c}}+
\frac{T_m-T_f}{R_{th,m}}-UA(\dot m,v)(T_f-T_{amb})+P_{pump}.
\]

Pompa debisi en sıcak hücre/motor sıcaklığına göre sürekli olarak modüle edilir. Kanal termal direnci debinin (0.8) kuvvetiyle; radyatör (UA) değeri debi ve araç hızıyla değişir.

## Üretilen grafikler

`plotSimulationResults` dört figür oluşturur:

1. hedef–gerçek hız ve ivme,
2. tork–hız zarfı üzerinde çalışma noktaları, field weakening noktaları ve d/q akımları,
3. paket SoC’si ile 96 seri-grubun gerilimleri,
4. hücre/motor/soğutucu sıcaklıkları, pompa debisi ve pasif dengeleme etkinliği.

Sonuç ve PNG kaydı varsayılan olarak kapalıdır. Açmak için simülasyondan önce konfigürasyonu programatik çalıştırın:

```matlab
cfg = defaultEVConfig("short");
cfg.output.saveResults = true;
cfg.output.saveFigures = true;
results = runEVSimulation(cfg);
plotSimulationResults(results, cfg);
```

## Kalibrasyon notu

Varsayılan LUT ve kayıp katsayıları tutarlı bir kontrol-tasarım başlangıç modelidir; belirli bir hücre, motor ve inverter için test verisiyle kalibre edilmelidir. Model anahtarlama olaylarını tek tek çözmez; 20 kHz inverterin anahtarlama kaybını ortalama enerji modeliyle hesaplar. Lastik kayması, yanal/yaw dinamiği, diferansiyel ve ayrıntılı CFD bu kapsamda değildir.
