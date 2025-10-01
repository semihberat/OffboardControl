# 🚁 PX4 ROS2 Offboard Control

PX4 ile ROS2 arası offboard kontrol paketi. Tek ve çoklu drone kontrolü için.

## 🚀 Hızlı Başlangıç

### Build
```bash
colcon build --packages-select px4_ros_com
source install/setup.bash
```

### Kullanım
```bash
# Tek drone
ros2 run px4_ros_com main --ros-args -p px4_namespace:="/fmu/"

# Çoklu drone
ros2 run px4_ros_com main --ros-args -p px4_namespace:="/px4_1/"
ros2 run px4_ros_com main --ros-args -p px4_namespace:="/px4_2/"
```

## 📁 Yapı

```
src/offboard/
├── controllers/
│   └── offboard_controller.hpp    # Ana kontrol sınıfı
└── main.cpp                       # Ana uygulama

launch/
└── multi_robot_start.xml          # Çoklu drone launch
```

## ⚙️ Özellikler

- ✈️ **Otomatik Kalkış**: Drone otomatik kalkar
- 🎯 **Waypoint Takibi**: Belirlenen noktaya gider  
- 🚁 **Çoklu Drone**: Aynı anda birden fazla drone
- 📡 **Gerçek Zamanlı**: GPS ve pozisyon takibi
- 🔧 **Namespace**: `/fmu/`, `/px4_1/`, `/px4_2/` desteği

## 🎮 Kontrol Mantığı

1. **10 setpoint** gönder → **Offboard mode**
2. **5 setpoint** sonra → **Arm** et
3. **Kalkış**: -5m yüksekliğe çık
4. **Navigasyon**: (5,5,-5) noktasına git

## 🔧 Parametreler

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `px4_namespace` | `"/fmu/"` | PX4 namespace |

## 📡 Topic'ler

```
# Komut gönder
/{namespace}/fmu/in/offboard_control_mode
/{namespace}/fmu/in/trajectory_setpoint
/{namespace}/fmu/in/vehicle_command

# Veri al  
/{namespace}/fmu/out/vehicle_local_position
/{namespace}/fmu/out/vehicle_gps_position
```

## 🐛 Sorun Giderme

```bash
# Topic'leri kontrol et
ros2 topic list | grep fmu

# Pozisyon verisi
ros2 topic echo /px4_1/fmu/out/vehicle_local_position

# Drone durumu
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1
```

---

**⚠️ Güvenlik**: Test ortamında kullanın, gerçek uçuşlarda dikkatli olun!