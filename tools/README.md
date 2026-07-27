# 정적 장애물 생성기 (obstacle_map_maker)

맵 이미지 위에 마우스로 정적 장애물을 배치하고, 장애물이 구워진(baked) 맵을
만들어 f1tenth 시뮬레이터(f1sim)를 바로 띄워 주는 GUI 도구입니다.

## 실행

```bash
python3 ~/f1tenth_gym/tools/obstacle_map_maker.py
```

ROS 환경 없이 실행 가능한 독립 도구입니다 (`python3-tk`, `Pillow`, `PyYAML` 필요).
시뮬레이터 실행 버튼만 ROS 2 워크스페이스(`~/f1tenth_gym/install`)를 사용합니다.

## 사용 순서

1. **맵 선택** — 상단 드롭다운에서 번들 맵(levine, Spielberg_map)을 고르거나
   `찾아보기…` 로 임의의 맵 yaml(예: `~/2026_IFAC/src/monte_carlo_localization/maps/*.yaml`)을 선택.
2. **장애물 배치** — 캔버스 좌클릭으로 장애물 추가.
   - 모양: 원(반지름[m]) / 사각형(가로x세로[m], 축 정렬)
   - 우클릭 = 근처 장애물 삭제, `Ctrl+Z` = 되돌리기, `전체 삭제` 버튼
   - 휠 = 줌, 휠클릭 드래그 또는 `Ctrl+드래그` = 화면 이동, `화면 맞춤` = 전체 보기
3. **시작 포즈(선택)** — `시작 포즈` 모드에서 클릭(위치) 후 드래그(방향).
   설정하지 않으면 `sim.yaml` 의 `sx/sy/stheta` 를 그대로 사용.
4. **적용 & 실행** — 아래 3개 파일을 원본 맵 폴더에 저장한 뒤 시뮬레이터를 실행.
   - `<이름>.png|.pgm` — 장애물이 검정(점유)으로 구워진 맵 이미지
   - `<이름>.yaml` — 원본 메타데이터를 복사하고 `image:` 만 교체한 맵 yaml
   - `<이름>.obstacles.yaml` — 장애물/시작 포즈 기록 (재편집용 사이드카)
   - 기본 출력 이름은 `<원본>_obs`. 같은 맵을 다시 열면 사이드카 복원을 제안.
   - `맵만 저장` 버튼은 시뮬레이터 실행 없이 저장만 수행.
5. **시뮬 종료 / 재적용** — `적용 & 실행` 을 다시 누르면 기존 시뮬을 끄고 새 맵으로
   재시작. `시뮬 종료` 버튼으로 수동 종료. 로그: `/tmp/obstacle_sim_launch.log`

## 시뮬레이터 실행 방식

내부적으로 아래 명령을 실행합니다 (새 launch 파일 사용):

```bash
source /opt/ros/jazzy/setup.bash   # 설치된 ROS 2 배포판 자동 감지
source ~/f1tenth_gym/install/setup.bash
ros2 launch f1tenth_gym_ros obstacle_sim_launch.py \
    map_path:=/abs/path/<이름> map_img_ext:=.png \
    sx:=0.0 sy:=0.0 stheta:=0.0 num_agent:=1 rviz:=true
```

`obstacle_sim_launch.py` 는 `gym_bridge_launch.py` 와 동일한 노드 구성이지만
맵 경로·시작 포즈·에이전트 수·RViz 여부를 launch 인자로 덮어쓸 수 있습니다.
비워 둔 인자는 `config/sim.yaml` 값을 그대로 사용합니다.

> 새 launch 파일을 처음 쓰기 전에 워크스페이스 빌드가 필요합니다:
> `cd ~/f1tenth_gym && ./install.sh` (또는 `colcon build --symlink-install --base-paths f1tenth_gym_ros`)

## 동작 원리

- 맵 yaml 의 `resolution`/`origin` 으로 월드(m) ↔ 픽셀 좌표를 변환하고,
  장애물을 그레이스케일 이미지에 검정(0, `negate: 1` 이면 255)으로 래스터라이즈합니다.
- f110_gym 의 C++ 코어와 nav2 map_server 모두 이미지의 어두운 픽셀을 점유로
  해석하므로, 구워진 장애물은 LiDAR 레이캐스팅·충돌 판정·RViz 맵 표시에 모두 반영됩니다.
- 원본 맵 파일은 절대 수정하지 않습니다 (원본 이름으로 저장 시도 시 차단).
