# Übung 2: Odometrie und TF (`odom_transform`)

## Starten

Jeder Befehl in einem eigenen Ubuntu-Terminal (`wsl -d Ubuntu-24.04`).
ROS und der Workspace werden über `~/.bashrc` gesourced.

```sh
# 1. Bagfile in Schleife, die TF des Roboters nach /tf_old umbenannt
ros2 bag play $(marina resolve dlg_feldtage_24:cut) --loop --remap /tf:=/tf_old

# 2. Unsere Node (Schritt 3, 4, 7)
ros2 run odom_transform odom_transform_node --ros-args -p frame_name:=my_odom
#    mit IMU für Drehungen auf der Stelle (Schritt 8, freiwillig)
ros2 run odom_transform odom_transform_node --ros-args -p frame_name:=my_odom -p use_imu:=true

# 3. TF-Baum (Schritt 5)
ros2 run rqt_tf_tree rqt_tf_tree

# 4. RViz (Schritt 6): Fixed Frame = my_odom, Add -> By topic -> /ouster/points PointCloud2,
#    Add -> By display type -> TF
rviz2
```

## Was die Node macht

Sie subscribed `/odom` (`nav_msgs/msg/Odometry`) und integriert `twist.twist.linear.x` (v) und `twist.twist.angular.z` (ω) über die Zeit.
`dt` kommt aus den Zeitstempeln der Nachrichten.
Wir nutzen die Mittelpunktregel, also den Winkel in der Mitte des Intervalls.
Das ist in Kurven genauer als der einfache Euler-Schritt:

```
θ_mid = θ + ω * dt / 2
x    += v * cos(θ_mid) * dt
y    += v * sin(θ_mid) * dt
θ    += ω * dt
```

- **Reset:** Springt der Zeitstempel zurück (Loop der Bagfile), wird die Pose auf den Ursprung gesetzt.
- **TF:** Die Pose wird als `my_odom -> <child_frame_id aus /odom>` gesendet.
- **Parameter:** `frame_name` (Standard `my_odom`), `use_imu` (Standard `false`), `standstill_speed` (Standard `0.05` m/s).
- **Log:** Alle 2 s wird die aktuelle Pose ausgegeben.

**Warum `TransformBroadcaster` statt `StaticTransformBroadcaster`?**
Das Blatt verlinkt den Static Broadcaster.
Statische Transformationen gehen auf `/tf_static` (latched) und sind für Frames gedacht, die sich nie bewegen, etwa Sensor-Halterungen.
Unsere Pose ändert sich mit jeder Odom-Nachricht, deshalb nutzen wir den normalen `TransformBroadcaster`.
Er published auf `/tf`, und genau dort liest auch RViz in Schritt 6.

**Schritt 8 (IMU, freiwillig):**
Mit `use_imu:=true` subscribed die Node zusätzlich `/imu/data_raw` (`sensor_msgs/msg/Imu`).
Solange `|linear.x| < standstill_speed` ist (Drehung auf der Stelle), nimmt sie für ω den Mittelwert der Gyro-Werte `angular_velocity` seit der letzten Odom-Nachricht statt `angular.z` der Räder.
Hinweis: Das Gyroskop misst Winkelgeschwindigkeit, nicht Winkelbeschleunigung.
Vorher wird der Gyro-Vektor mit der statischen Transformation aus `/tf_static` vom IMU-Frame in den Roboter-Frame gedreht.
Ist die IMU gedreht oder kopfüber montiert, stimmt so trotzdem das Vorzeichen der Gierrate.
Grund: Beim Drehen auf der Stelle rutschen die Räder stark, die Rad-Odometrie überschätzt oder unterschätzt dann die Drehung.
Das Gyroskop misst die echte Drehrate, hat aber einen Bias, der über längere Zeit driftet.

Messung mit `dlg_feldtage_24:cut` (40 s, offline nachgerechnet):

| Variante | Endpose x, y | Endwinkel |
|---|---|---|
| Nur Rad-Odometrie (unsere Node) | 3.38 m, −26.37 m | −86.6° |
| Pose des Roboters selbst (`pose.pose` in `/odom`) | 3.38 m, −26.37 m | −86.6° |
| IMU bei `abs(v) < 0.001` (wörtlich `linear.x == 0`) | 3.25 m, −26.38 m | −86.8° |
| IMU bei `abs(v) < 0.05` (Standard) | 1.76 m, −26.37 m | −90.1° |
| Gyroskop allein über 40 s | | −90.5° |

- Unsere Integration stimmt exakt mit der Odometrie des Roboters überein.
- Die Räder unterschätzen die Drehung um etwa 4°, das ist der Schlupf beim Drehen auf der Stelle.
- Beim Drehen melden die Räder trotzdem etwa 0.02 m/s. Mit wörtlich `== 0` greift die IMU deshalb fast nie, daher der Standard `0.05` m/s.
- 4° Winkelfehler bedeuten nach 25 m Fahrt schon etwa 1.6 m seitliche Abweichung.

## Fragen

**Frage 1: Probleme der Odometrie beim differenziell angetriebenen Roboter**
- **Schlupf:** Räder rutschen auf Gras, Erde oder nassem Boden (die Bagfile ist von den DLG-Feldtagen, also draußen auf dem Feld). Beim Drehen auf der Stelle rutschen die Räder seitlich, besonders bei Skid-Steering mit vier Rädern. Der Roboter in der Bagfile ist so einer: In seinem `/tf` hängen vier Radframes (`front_left`, `front_right`, `rear_left`, `rear_right_wheel_link`) an `chassis_link`.
- **Kalibrierfehler:** Abweichungen bei Raddurchmesser und Spurbreite (Luftdruck, Last, Verschleiß) verfälschen v und ω systematisch.
- **Unebener Boden:** Wir rechnen nur in 2D. Steigungen und Bodenwellen werden als Vorwärtsfahrt gezählt.
- **Drift:** Die Integration hat keine absolute Korrektur, der Fehler wächst unbegrenzt. Ein kleiner Winkelfehler wird mit der gefahrenen Strecke zu einem immer größeren Positionsfehler.
- **Diskretisierung:** Diskrete Integration, verlorene oder verspätete Nachrichten und Zeitstempel-Jitter führen zu Fehlern in `dt`.

**Frage 2: Welches QoS, welche Reliability für den Treiber?**
Für einen hochfrequenten Strom von Geschwindigkeiten bzw. Odometrie: **Best Effort**, `Volatile`, `Keep Last` mit kleiner Tiefe (`rclcpp::SensorDataQoS()`).
Ein neuer Wert macht den alten wertlos.
Erneutes Senden bei Reliable kostet Latenz und Bandbreite, gerade über WLAN zum Roboter, und eine einzelne verlorene Nachricht ist verschmerzbar.
Gegenargument: Ein Subscriber, der integriert (wie unsere Node), verliert bei einer fehlenden Nachricht ein Stück Bewegung.
Bei niedriger Rate oder wenn jeder Wert zählt, ist Reliable vertretbar.
Wichtig für die Kompatibilität: Ein Best-Effort-Subscriber kann von Reliable- und von Best-Effort-Publishern lesen.
Ein Reliable-Subscriber bekommt von einem Best-Effort-Publisher gar nichts.
Deshalb subscribed unsere Node mit `SensorDataQoS()`.
Die QoS der Bagfile zeigt `ros2 topic info /odom --verbose`.
