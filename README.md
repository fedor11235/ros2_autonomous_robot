# Автономный мобильный робот — ROS 2 + Gazebo

Автономная навигация мобильного робота с дифференциальным приводом в симуляторе
Gazebo на базе ROS 2. Робот едет из **стартовой** зоны в **финишную** через поле
препятствий, **не касаясь их**, используя 2D-лидар, RGB-D камеру глубины и GPS.

> **Контекст (тестовое задание).** Проект демонстрирует моделирование робота
> (URDF/Xacro + плагины Gazebo), настройку сцены, ROS-архитектуру, автономную
> навигацию с объездом препятствий, интеграцию GPS, качество кода и
> воспроизводимость (запуск одной командой). Соответствие критериям оценки —
> в конце этого файла.

| | |
|---|---|
| **ROS** | ROS 2 Humble |
| **Симулятор** | Gazebo Classic 11 (`gazebo_ros_pkgs`) |
| **Робот** | Кастомный diff-drive URDF/Xacro (`autobot`) |
| **Сенсоры** | 2D-лидар (`/scan`), RGB-D камера глубины (`/camera/*`), GPS (`/gps/fix`) |
| **Навигация** | Кастомный проход по точкам + реактивный объезд по лидару |
| **Запуск** | Одна команда (нативно `./run.sh` или Docker) |

---

## Быстрый старт

### Вариант A — Docker (максимально воспроизводимо)

Нужны Docker и, для GUI, X-сервер (Linux).

```bash
git clone https://github.com/fedor11235/ros2_autonomous_robot.git
cd ros2_autonomous_robot

xhost +local:root            # разрешить контейнеру доступ к X-серверу
docker compose up --build    # соберёт образ и запустит всё целиком
```

Без GUI (headless, например для CI / удалённого запуска):

```bash
docker compose run autobot ros2 launch robot_bringup bringup.launch.py gui:=false rviz:=false
```

### Вариант B — нативный ROS 2 Humble

Нужны ROS 2 Humble + Gazebo Classic (`sudo apt install ros-humble-desktop
ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-xacro`).

```bash
git clone https://github.com/fedor11235/ros2_autonomous_robot.git
cd ros2_autonomous_robot
./run.sh                     # при первом запуске соберёт воркспейс, затем стартует
```

Эта единственная команда поднимает **Gazebo + робота + RViz + автономную
навигацию**. Робот сразу начинает движение по маршруту к красной финишной зоне,
объезжая препятствия.

Полезные варианты:

```bash
./run.sh gui:=false          # Gazebo без GUI (RViz остаётся)
./run.sh autonomy:=false     # только симуляция, ручное управление (см. ниже)
```

---

## Что вы должны увидеть

1. Открывается Gazebo: арена 10×10 м со стенами, **зелёная стартовая площадка**
   в начале координат, **красная финишная площадка** в точке `(4, 4)` и
   оранжевые/синие препятствия между ними.
2. В RViz видны модель робота, данные лидара в реальном времени, запланированный
   маршрут по точкам (зелёная линия + сферы) и след одометрии.
3. Робот едет от старта к финишу, замедляясь и объезжая каждое препятствие, и
   останавливается на финишной площадке. В терминале появляется
   `Final waypoint reached. Goal complete.`, а нода GPS логирует уменьшающееся
   расстояние до цели.

---

## Демо

🎥 **Видео заезда (1–3 мин, Gazebo + RViz):** _ссылку вставить сюда_
<!-- Замените строку выше на реальную ссылку, например:
     [Смотреть демо на YouTube](https://youtu.be/XXXXXXXXXXX) -->

Как записать видео и rosbag — пошагово в [`docs/demo.md`](docs/demo.md).

---

## Ручное управление (опционально)

С `autonomy:=false` роботом можно управлять вручную:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # публикует /cmd_vel
```

---

## Запись демо / rosbag

```bash
# Записать всё необходимое для повторного воспроизведения заезда:
ros2 bag record -o demo_bag /tf /tf_static /odom /scan /cmd_vel \
    /gps/fix /gps/odom /planned_path /waypoint_markers /camera/depth/image_raw

# Воспроизвести позже:
ros2 bag play demo_bag
```

Чек-лист для записи видео — в [`docs/demo.md`](docs/demo.md).

---

## Структура репозитория

```
ros2_autonomous_robot/
├── run.sh                      # запуск одной командой (нативно)
├── Dockerfile / docker-compose.yml
├── Makefile                    # сокращения: сборка / тесты / запуск
├── docs/
│   ├── architecture.md         # ноды, топики, алгоритм управления, дерево TF
│   ├── electronics.md          # бонусные вопросы по железу (RPi 5, питание, тесты)
│   └── demo.md                 # как записать видео + rosbag
└── src/
    ├── robot_description/      # URDF/Xacro, сенсоры, конфиг RViz
    ├── robot_gazebo/           # мир (препятствия, старт/финиш, геореференс GPS) + спавн
    ├── robot_navigation/       # навигатор по точкам, GPS-локализатор, параметры, тесты
    └── robot_bringup/          # верхнеуровневый запуск одной командой
```

Подробности по каждой ноде и топику — в
[`docs/architecture.md`](docs/architecture.md).

---

## Ключевые топики

| Топик | Тип | Направление |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | навигатор → робот |
| `/odom` | `nav_msgs/Odometry` | робот → навигатор |
| `/scan` | `sensor_msgs/LaserScan` | лидар → навигатор |
| `/gps/fix` | `sensor_msgs/NavSatFix` | сенсор GPS |
| `/gps/odom` | `nav_msgs/Odometry` | GPS-локализатор (локальный ENU) |
| `/planned_path`, `/waypoint_markers` | path / markers | навигатор → RViz |
| `/camera/depth/image_raw`, `/camera/points` | image / cloud | камера глубины |

---

## Тесты

Юнит-тесты чистой логики (геометрия/хелперы контроллера) запускаются без симулятора:

```bash
colcon test --packages-select robot_navigation
colcon test-result --verbose
# или:  cd src/robot_navigation && python3 -m pytest test -v
```

---

## Конфигурация

Точки маршрута и коэффициенты контроллера лежат в
[`src/robot_navigation/config/nav_params.yaml`](src/robot_navigation/config/nav_params.yaml).
Чтобы изменить маршрут, отредактируйте список `waypoints` (плоский
`[x0, y0, x1, y1, ...]` в кадре `odom`); при запуске через `./run.sh` с
`--symlink-install` пересборка не требуется.

---

## Соответствие критериям оценки (11 баллов)

| Критерий | Где реализовано |
|---|---|
| Корректность URDF/плагинов (2) | `robot_description/urdf/*.xacro` — валидный URDF, корректные инерции, плагины diff-drive/lidar/depth/GPS |
| Конфигурация мира Gazebo (1) | `robot_gazebo/worlds/course.world` — плоскость, стены, препятствия, стартовая/финишная зоны, геореференс для GPS |
| ROS-архитектура (2) | Разделение на пакеты description/gazebo/navigation/bringup; чистые топики и launch-файлы |
| Навигация и объезд (3) | `robot_navigation/waypoint_navigator.py` — go-to-goal + реактивный объезд по лидару |
| GPS-функционал (1) | GPS-сенсор в URDF/мире + `gps_localizer.py` (NavSatFix → локальный ENU) |
| Качество кода (1) | Докстринги, модульность, unit-тесты, линт-зависимости |
| Воспроизводимость (1) | `./run.sh` и Docker — запуск одной командой, зафиксированные версии |

Бонусные вопросы по железу (подключение датчиков к Raspberry Pi 5, схема питания,
автономное тестирование сенсоров) разобраны в
[`docs/electronics.md`](docs/electronics.md).

---

## Лицензия

MIT — см. [LICENSE](LICENSE).
