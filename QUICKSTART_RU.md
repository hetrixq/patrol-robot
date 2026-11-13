# Patrol RBM v5 — полная инструкция для вашего робота

## Что поменялось

YOLO полностью убрана: `ultralytics`, `torch`, NCNN не загружаются.
Существующий SLAM, Nav2, LiDAR, RTSP/MediaMTX и мост Lichtblick оставляем.
Кнопка **CHECK** имитирует обнаружение человека. Никакой проверки ног камерой
теперь нет: оператор сам решает, когда провести проверку пропуска.

## 0. Установи новую папку (на хосте робота)

Если новые файлы **уже закоммичены в ваш PATROL-ROBOT на GitHub**:

```bash
cd ~/robomarvel/PATROL-ROBOT
git pull --ff-only
ls -l rbm_patrol_v5/start.sh
```

Если используешь архив `PATROL_RBM_v5.zip`, передай его на робот в
`~/robomarvel/PATROL-ROBOT/`, затем на **хосте**:

```bash
cd ~/robomarvel/PATROL-ROBOT
python3 -m zipfile -e PATROL_RBM_v5.zip .
ls -l rbm_patrol_v5/start.sh
```

**Не нужно** `prepare_project.sh`, `install_on_rbm.sh`, `setup.sh`, скачивание
`robomarv_26` или пересоздание Docker.

## 1. Войти в существующий ROS Docker

```bash
docker exec -it ros bash
cd /src/PATROL-ROBOT/rbm_patrol_v5
chmod +x *.sh
```

Перед стартом **останови** предыдущие `start_classroom.sh`, v4 `start.sh` и
их `route_manager/leg_detector`. Не запускай две патрульные ноды одновременно.

## 2. Проверить зависимости

```bash
./preflight.sh
```

Ожидаем `Result: 0 errors; YOLO is neither required nor imported`.
Нужен установленный `cv2` из вашего `/root/venv`, а не NCNN-модель.

## 3. Запустить (пока без движения)

```bash
./start.sh
```

Ожидаем `PATROL v5 started: NO YOLO` и `[IDLE]`.
Не надо запускать другой ROS/SLAM pipeline, если он уже работает.

Во втором окне Docker:

```bash
cd /src/PATROL-ROBOT/rbm_patrol_v5
./patrolctl.sh status
ros2 topic list -t | grep /patrol/
```

Ожидаем `data: IDLE` и `/patrol/camera/image/compressed` типа
`sensor_msgs/msg/CompressedImage`.

## 4. Импорт Lichtblick

1. Открой Lichtblick и подключись по `ws://IP_РОБОТА:8765`.
2. `Layouts → Import from file` → `lichtblick/PATROL_RBM_v5.json`.
3. Слева 3D-карта, справа `QR CAMERA` (Image panel), ниже статус/события,
   внизу кнопки START, STOP, CHECK, RESET, CAMERA ON/OFF и другие.
4. Если Image panel пуст, это **нормально**, пока камера закрыта.

Если панель после импорта не выбрала источник автоматически, нажми шестерёнку
на Image panel и выбери **Topic** = `/patrol/camera/image/compressed`.
Lichtblick поддерживает ROS 2 `sensor_msgs/msg/CompressedImage` напрямую.

## 5. Проверить QR и видео без движения

1. Убедись, что в статусе `IDLE`.
2. Нажми **CAMERA ON** в раскладке Lichtblick.
3. Изображение появится в Image panel примерно 2 раза в секунду.
4. Покажи в камеру `tools/PERSON_001.png` (на экране смартфона или бумаге).
5. Зелёная рамка + подпись `QR: PERSON_001` = код прочитан.
6. Жёлтая рамка = контур найден, но код не декодирован: поднеси ближе,
   разверни экран и убери блики. Без рамки = QR пока не найден.
7. Статус `/patrol/qr/status` покажет `PREVIEW: QR PERSON_001`.
8. Нажми **CAMERA OFF** — RTSP закроется, чтобы снизить нагрузку.

**Важно:** PREVIEW ничего не разрешает и не запрещает. Это только камера.

## 6. Проверить кнопку CHECK без движения

1. Оставь робот в `IDLE`, маршрут и карты пока вообще не нужны.
2. Нажми в Lichtblick **CHECK**.
3. Статус: `CHECKING`, камера автоматически включится.
4. После первого кадра начнётся отсчёт 15 секунд.
5. Покажи `PERSON_001` -> `GRANTED`, затем `IDLE`.
6. Нажми **CHECK** снова и покажи `PERSON_999` -> `DENIED`, сирена.
7. Нажми **RESET** -> `IDLE`, сирена прекращается.
8. Снова **CHECK**, ничего не показывай 15 секунд -> `DENIED`.
9. Снова **RESET**.

В терминале альтернативно:

```bash
./patrolctl.sh camera_on
./patrolctl.sh camera_off
./patrolctl.sh check
./patrolctl.sh reset
```

Отдельный тест фейкового топика:

```bash
ros2 topic pub --once /patrol/manual_check std_msgs/msg/Bool '{data: true}'
```

## 7. Проверить патруль с Click Point

Только когда основной SLAM/Nav2 стабильно работает, скорость небольшая,
есть свободная площадка и у оператора доступен физический аварийный выключатель.

1. Поездить вручную, пока карта существует в живом SLAM. В файл не сохраняем.
2. В 3D panel выбери Click Point (не Goal/Pose), топик `/clicked_point`,
   координатная система `map`.
3. Отметь 2–10 безопасных точек, лучше далеко от столов и людей.
4. Нажми **LOOP**, затем **START**.
5. Робот едет по точкам без YOLO и без камеры.
6. Во время движения нажми **CHECK**. Должно стать `STOPPING`.
7. Только после подтверждения завершения Nav2 статус станет `CHECKING`,
   камера покажет QR. Если цель не отменяется, будет `SAFETY_FAULT`;
   не продолжай тест до проверки физической остановки.
8. Покажи `PERSON_001`, чтобы продолжить патруль.
9. Откажи в пропуске / не показывай код 15 секунд — будет `DENIED`, сирена.
10. **RESET** снимает блокировку, но не начинает движение автоматически;
    для нового прохода нажми START.

Нужно остановить движение:

```bash
./patrolctl.sh stop
```

И **обязательно убедиться**, что колёса физически остановились. Для опасной
ситуации используется **физический аварийный выключатель**, не кнопка ПО.

## 8. Если видеокамера не показывает картинку

Внутри контейнера:

```bash
./patrolctl.sh qr
ros2 topic info /patrol/camera/image/compressed -v
ros2 topic echo /patrol/qr/status --once --qos-durability transient_local
```

`CAMERA ON` должен открыть один RTSP-поток `rtsp://localhost:8554/cam`.
Удостоверься, что RTSP сейчас работает в вашем RBM MediaMTX.
В Preview QR читается только на 2 FPS, чтобы не перегружать Pi.

Если Raspberry Pi снова выключается с красным светодиодом — это уже не
признак ошибки YOLO (её нет). Проверяй питание и системные журналы на хосте.

## 9. Типовые состояния

| Состояние | Камера | Движение |
|---|---|---|
| IDLE | OFF или PREVIEW по кнопке | Нет |
| PATROL | OFF | По маршруту Nav2 |
| STOPPING | OFF | Отменяется цель Nav2 |
| CHECKING | QR + Image panel (2 FPS) | Нет новой цели |
| DENIED | OFF, сирена до RESET | Нет новой цели |
| STOPPED | OFF или PREVIEW | Нет |
| SAFETY_FAULT | OFF | Проверить физический стоп |

Проверка всегда ручная через CHECK. Это демонстрация, не автономное
обнаружение людей и не средство предотвращения столкновений.
