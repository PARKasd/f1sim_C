# sim.yaml 차량 및 시뮬레이션 파라미터

이 문서는 `config/sim.yaml`에서 수정할 수 있는 차량 동역학 파라미터와
시뮬레이션 관련 파라미터를 설명합니다.

값은 ROS 노드가 시작될 때 읽힙니다. 따라서 `sim.yaml`을 수정한 뒤에는
시뮬레이터를 다시 실행하면 다음 실행부터 반영됩니다. `--symlink-install` 없이
빌드한 환경에서 설치 경로의 YAML을 사용한다면 `colcon build`를 다시 실행해야 할
수 있습니다.

## YAML 위치

```yaml
bridge:
  ros__parameters:
    timestep: 0.01
    integrator: 'RK4'

    vehicle:
      mu: 1.0489
      C_Sf: 4.718
      C_Sr: 5.4562
      lf: 0.15875
      lr: 0.17145
      h: 0.074
      m: 3.74
      I: 0.04712
      s_min: -0.4189
      s_max: 0.4189
      sv_min: -3.2
      sv_max: 3.2
      v_switch: 7.319
      a_max: 9.51
      v_min: -5.0
      v_max: 20.0
      width: 0.31
      length: 0.58
```

시뮬레이터는 single-track bicycle model을 사용합니다. 입력은 목표 조향각과
목표 종방향 속도이며, 내부 컨트롤러가 아래 제한을 적용해 실제 조향 속도와
종방향 가속도로 변환합니다.

## 차량 동역학 파라미터

| 이름 | 단위 | 의미 | 튜닝 시 영향 |
| --- | --- | --- | --- |
| `mu` | 없음 | 타이어와 노면 사이의 마찰 계수입니다. | 값이 크면 미끄러지기 전까지 더 큰 횡력을 낼 수 있습니다. 값이 작으면 언더스티어 또는 슬립이 더 쉽게 발생합니다. |
| `C_Sf` | 모델 강성 계수 | 앞 타이어 코너링 강성입니다. | 값이 크면 같은 slip angle에서 앞 타이어 횡력이 커집니다. 조향 초기 반응과 yaw 응답이 달라집니다. |
| `C_Sr` | 모델 강성 계수 | 뒤 타이어 코너링 강성입니다. | 값이 크면 뒤쪽이 더 안정적으로 버팁니다. 값이 작으면 차량이 더 쉽게 회전하거나 오버스티어 성향이 커질 수 있습니다. |
| `lf` | m | 차량 무게중심에서 앞 차축까지의 거리입니다. | `lf + lr`이 wheelbase가 됩니다. 앞뒤 무게중심 배치와 yaw dynamics에 영향을 줍니다. |
| `lr` | m | 차량 무게중심에서 뒤 차축까지의 거리입니다. | `lf + lr`이 wheelbase가 됩니다. 앞뒤 무게중심 배치와 yaw dynamics에 영향을 줍니다. |
| `h` | m | 차량 무게중심 높이입니다. | 가속과 제동 중 전후 하중 이동량에 영향을 줍니다. |
| `m` | kg | 차량 질량입니다. | 값이 크면 타이어 힘에 대한 속도와 yaw 반응이 느려집니다. 값이 작으면 같은 입력에서 더 민감하게 움직입니다. |
| `I` | kg*m^2 | 수직축 기준 yaw 관성 모멘트입니다. | 값이 크면 yaw rate 변화가 느려집니다. 값이 작으면 차량이 더 빠르게 회전합니다. |
| `s_min` | rad | 최소 조향각입니다. 보통 음수입니다. | 조향각 하한을 제한합니다. 이 시뮬레이터의 convention에서는 한쪽 방향 최대 조향각입니다. |
| `s_max` | rad | 최대 조향각입니다. 보통 양수입니다. | 조향각 상한을 제한합니다. 이 시뮬레이터의 convention에서는 반대쪽 방향 최대 조향각입니다. |
| `sv_min` | rad/s | 최소 조향 속도입니다. 보통 음수입니다. | 조향각이 감소할 수 있는 최대 속도를 제한합니다. |
| `sv_max` | rad/s | 최대 조향 속도입니다. 보통 양수입니다. | 조향각이 증가할 수 있는 최대 속도를 제한합니다. |
| `v_switch` | m/s | 고속 영역에서 양의 가속도를 줄이기 시작하는 속도입니다. | 이 속도보다 빠르면 내부 컨트롤러의 가속 가능량이 줄어듭니다. 최고속 근처 가속감을 조정할 때 사용합니다. |
| `a_max` | m/s^2 | 내부 컨트롤러가 사용할 수 있는 최대 가속도 크기입니다. | 가속과 제동의 최대 크기를 제한합니다. 값이 크면 속도 명령을 더 빨리 따라갑니다. |
| `v_min` | m/s | 허용되는 최소 종방향 속도입니다. 보통 음수입니다. | 후진 최고속과 action space의 속도 하한을 결정합니다. |
| `v_max` | m/s | 허용되는 최대 종방향 속도입니다. | 전진 최고속과 action space의 속도 상한을 결정합니다. |
| `width` | m | 차량 차체 폭입니다. | 차량 간 충돌 판정과 LiDAR의 차량 형상 처리에 사용됩니다. |
| `length` | m | 차량 차체 길이입니다. | 차량 간 충돌 판정과 LiDAR의 차량 형상 처리에 사용됩니다. |

## 물리 적분 파라미터

| 이름 | 단위 | 의미 | 튜닝 시 영향 |
| --- | --- | --- | --- |
| `timestep` | s | 물리 업데이트 한 스텝의 시간 간격입니다. | 값이 작으면 더 정확하지만 CPU 사용량이 증가합니다. 값이 크면 빠르지만 고속 주행이나 충돌 근처에서 오차가 커질 수 있습니다. |
| `integrator` | 문자열 | 차량 상태 적분 방식입니다. 지원 값은 `RK4`, `Euler`입니다. | `RK4`는 더 정확하고 기본값으로 권장됩니다. `Euler`는 단순하고 빠르지만 큰 `timestep`에서 오차가 커질 수 있습니다. |

## LiDAR 및 transform 파라미터

| 이름 | 단위 | 의미 | 튜닝 시 영향 |
| --- | --- | --- | --- |
| `scan_distance_to_base_link` | m | `base_link` 기준 LiDAR의 x축 방향 위치입니다. | LiDAR가 차량 앞쪽 또는 뒤쪽에 얼마나 떨어져 있는지 결정합니다. |
| `scan_fov` | rad | LiDAR 시야각입니다. | 값이 크면 더 넓은 각도를 스캔합니다. |
| `scan_beams` | 개수 | 한 scan에 포함되는 beam 수입니다. | 값이 크면 각도 해상도가 높아지지만 연산량이 증가합니다. |

## 맵 파라미터

| 이름 | 단위 | 의미 |
| --- | --- | --- |
| `map_path` | 경로 | 확장자를 제외한 맵 YAML/이미지 파일 경로입니다. 예: `maps/levine` |
| `map_img_ext` | 문자열 | 맵 이미지 확장자입니다. 예: `.png` |

`map_path`는 같은 이름의 YAML과 이미지가 함께 있다고 가정합니다. 예를 들어
`map_path: '/path/levine'`, `map_img_ext: '.png'`이면 `/path/levine.yaml`과
`/path/levine.png`를 사용합니다.

## 차량 수와 시작 pose

| 이름 | 단위 | 의미 |
| --- | --- | --- |
| `num_agent` | 개수 | 시뮬레이션 차량 수입니다. `1` 또는 `2`를 사용합니다. |
| `sx` | m | ego 차량 시작 x 위치입니다. |
| `sy` | m | ego 차량 시작 y 위치입니다. |
| `stheta` | rad | ego 차량 시작 yaw 각도입니다. |
| `sx1` | m | opponent 차량 시작 x 위치입니다. |
| `sy1` | m | opponent 차량 시작 y 위치입니다. |
| `stheta1` | rad | opponent 차량 시작 yaw 각도입니다. |

시작 pose는 맵의 global coordinate frame 기준입니다.

## 토픽 및 namespace 파라미터

| 이름 | 의미 |
| --- | --- |
| `ego_namespace` | ego 차량 namespace입니다. |
| `ego_scan_topic` | ego 차량 LiDAR scan 토픽 이름입니다. |
| `ego_odom_topic` | ego 차량 odometry 토픽 이름입니다. |
| `ego_opp_odom_topic` | ego 기준 opponent odometry 토픽 이름입니다. |
| `ego_drive_topic` | ego 차량 drive command 토픽 이름입니다. |
| `opp_namespace` | opponent 차량 namespace입니다. |
| `opp_scan_topic` | opponent 차량 LiDAR scan 토픽 이름입니다. |
| `opp_odom_topic` | opponent 차량 odometry 토픽 이름입니다. |
| `opp_ego_odom_topic` | opponent 기준 ego odometry 토픽 이름입니다. |
| `opp_drive_topic` | opponent 차량 drive command 토픽 이름입니다. |

기존 ROS launch, RViz 설정, agent 코드가 이 이름들을 사용하므로 특별한 이유가
없으면 기본값을 유지하는 것이 안전합니다.

## Teleop 파라미터

| 이름 | 의미 |
| --- | --- |
| `kb_teleop` | `True`이면 keyboard teleop 노드를 같이 실행하도록 launch에서 사용합니다. |

## 튜닝 체크리스트

- `s_min < 0 < s_max`, `sv_min < 0 < sv_max`, `v_min < v_max` 형태를 유지합니다.
- `lf + lr`은 bicycle model의 wheelbase로 쓰입니다.
- 차량이 너무 둔하면 `m`, `I`, `a_max`, `sv_max`, `C_Sf`, `C_Sr`을 확인합니다.
- 차량이 너무 쉽게 회전하면 `C_Sf`, `C_Sr`, `lf`, `lr`, `I`, `mu`를 확인합니다.
- 최고속 또는 RL action 범위를 바꾸려면 `v_min`, `v_max`를 수정합니다.
- 충돌 판정 크기를 바꾸려면 `width`, `length`를 수정합니다.
